"""ROS side of the debug GUI.

Owns the node and every subscription. Subscription callbacks do ONE thing:
cache the newest message. All unpacking (bitfields, array indices) and all
derived math (error percentages, motor splits) lives in the methods below,
so the Qt layer never has to know a message format.

The GUI reads these methods; it never touches ROS.
"""

import math
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from crazyflie_interfaces.msg import Status, LogDataGeneric
from crazyflie_debug_interfaces.msg import DebugFlags
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray


# A value older than this is treated as "no data" rather than shown stale.
STALE_AFTER_SEC = 2.0

# Motor PWM is a 16-bit value: 0..65535. At 65535 the mixer has no headroom
# left, so the controller can no longer correct attitude - that is the ceiling
# seen in the crash bags, not a soft limit.
MOTOR_MAX_PWM = 65535

# Full-scale values: what counts as a 100% bar. Tuned from the flight notes -
# healthy hover sits near zero on all of these, and these numbers are roughly
# "this is already very wrong".
POSITION_ERROR_FULL_SCALE_M = 0.20    # 2 cm is normal tracking error
ATTITUDE_ERROR_FULL_SCALE_DEG = 30.0  # the runaway flights sat ~27-30 deg off
MOTOR_SPLIT_FULL_SCALE = 20000.0      # <2k healthy; ~20k was the M1 crash


def _quat_to_rpy(q):
    """Quaternion -> (roll, pitch, yaw) in radians, ZYX convention."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))  # clamp: asin blows up past +/-1
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _quat_is_valid(q):
    """In position-only mode the bridge sends NaN in w, and crazyflie_server
    uses that as the 'no orientation' flag. Don't do math on it."""
    return not (math.isnan(q.w) or math.isnan(q.x)
                or math.isnan(q.y) or math.isnan(q.z))


def _wrap_pi(angle):
    """Fold an angle difference into [-pi, pi] so 359 deg vs 1 deg reads as 2."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _drone_name_from_argv():
    """Pull `-p drone_name:=X` out of the command line, before any node exists."""
    import sys
    for arg in sys.argv:
        if arg.startswith('drone_name:='):
            return arg.split(':=', 1)[1]
    return None


def _pct(value, full_scale):
    """Scale to a 0-100 bar value, clamped."""
    return max(0.0, min(100.0, 100.0 * value / full_scale))


class TelemetryModel(Node):

    def __init__(self, drone_name=None, **kwargs):
        # The node name must be fixed at construction - ROS can't rename later,
        # and the drone_name parameter only arrives after super().__init__().
        # So sniff it from the command line first, and encode it into the node
        # name so two GUIs never collide. Under launch, the Node action's
        # name= overrides this anyway.
        if drone_name is None:
            drone_name = _drone_name_from_argv() or 'cf09'
        super().__init__(f'{drone_name}_debug_gui', **kwargs)
        self.declare_parameter('drone_name', drone_name)
        # fake_data:=true synthesises telemetry for working on the GUI with no
        # drone and no mocap. Commands are still published for real, because the
        # controller server validates them against ITS OWN tf - so a fake
        # altitude here can't talk a real drone into taking off.
        self.declare_parameter('fake_data', False)
        self.drone_name = self.get_parameter('drone_name').value
        self.fake_data = self.get_parameter('fake_data').value

        # depth=1 so a slow GUI can never build a backlog: the middleware keeps
        # only the newest sample and drops the rest.
        # crazyflie_server creates its publishers with a plain depth -> RELIABLE.
        qos = QoSProfile(depth=1,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)

        # /poses is published BEST_EFFORT by vicon_bridge.py (and read the same
        # way by crazyflie_server's SensorDataQoS). A RELIABLE subscriber would
        # silently never match it, so this one needs its own profile.
        poses_qos = QoSProfile(depth=1,
                               history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.BEST_EFFORT)

        # Newest message off each topic, and when it arrived (None = never).
        self.status = None           # crazyflie_interfaces/Status
        self.status_time = None
        self.motor_pwm = None        # crazyflie_interfaces/LogDataGeneric
        self.motor_pwm_time = None
        self.pose = None             # geometry_msgs/PoseStamped (EKF estimate)
        self.pose_time = None
        self.vicon_poses = None      # motion_capture_tracking_interfaces/NamedPoseArray
        self.vicon_poses_time = None

        if self.fake_data:
            self.get_logger().warn(
                'fake_data:=true - telemetry is SYNTHETIC. Commands are still '
                'published; the controller server validates them itself.')
            self._fake_t = 0.0
            self.create_timer(0.05, self._tick_fake)
        else:
            self.create_subscription(
                Status, f'/{self.drone_name}/status', self.on_status, qos)
            self.create_subscription(
                LogDataGeneric, f'/{self.drone_name}/motor_pwm', self.on_motor_pwm, qos)
            self.create_subscription(
                PoseStamped, f'/{self.drone_name}/pose', self.on_pose, qos)
            self.create_subscription(
                NamedPoseArray, '/poses', self.on_vicon_poses, poses_qos)

        # Last-resort pose source. crazyflie_sim publishes neither /<name>/pose
        # nor /poses -- its only output is TF -- so in sim both readouts above
        # are permanently None and this is the one that answers.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Commands out. RELIABLE with a little depth: a dropped takeoff is bad,
        # a dropped ESTOP is unacceptable, and these are rare human-paced events
        # so there is no backlog risk.
        self.command_pub = self.create_publisher(
            DebugFlags, f'/{self.drone_name}/debug_command', 10)

    # ------------------------------------------------------------------
    # Subscription callbacks. Each one only caches - no widgets, no math.
    # ------------------------------------------------------------------

    def on_status(self, msg):
        self.status = msg
        self.status_time = time.monotonic()

    def on_motor_pwm(self, msg):
        self.motor_pwm = msg
        self.motor_pwm_time = time.monotonic()

    def on_pose(self, msg):
        self.pose = msg
        self.pose_time = time.monotonic()

    def on_vicon_poses(self, msg):
        self.vicon_poses = msg
        self.vicon_poses_time = time.monotonic()

    # ------------------------------------------------------------------
    # Fake source. It builds real message objects and feeds them through the
    # callbacks above, so the unpacking, derived math and staleness logic are
    # all exercised exactly as they are with a live drone.
    # ------------------------------------------------------------------

    def _tick_fake(self):
        from motion_capture_tracking_interfaces.msg import NamedPose
        self._fake_t += 0.05
        t = self._fake_t

        status = Status()
        status.battery_voltage = max(3.4, 4.05 - 0.004 * t)
        status.supervisor_info = (Status.SUPERVISOR_INFO_IS_ARMED
                                  | Status.SUPERVISOR_INFO_IS_FLYING)
        status.rssi = 55
        self.on_status(status)

        # A slow one-way M1 drift, like the bent shaft - and clamped to the real
        # 16-bit ceiling so it saturates instead of inventing impossible values.
        base = 45000.0 + 3000.0 * math.sin(t / 3.0)
        # float() matters: LogDataGeneric.values rejects ints, and a bare
        # min(MOTOR_MAX_PWM, x) hands back the int constant once it clamps.
        def clamp(v):
            return float(max(0.0, min(float(MOTOR_MAX_PWM), v)))

        pwm = LogDataGeneric()
        pwm.values = [clamp(base + 120.0 * t),
                      clamp(base - 200.0),
                      clamp(base - 24.0 * t),
                      clamp(base + 150.0)]
        self.on_motor_pwm(pwm)

        yaw = math.radians(0.25 * t)
        pose = PoseStamped()
        pose.pose.position.x = 0.02 * math.sin(t)
        pose.pose.position.y = 0.01 * math.cos(t)
        pose.pose.position.z = 1.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.on_pose(pose)

        named = NamedPose()
        named.name = self.drone_name
        named.pose.position.x = 0.0
        named.pose.position.y = 0.0
        named.pose.position.z = 1.0
        named.pose.orientation.w = 1.0
        array = NamedPoseArray()
        array.poses = [named]
        self.on_vicon_poses(array)

    # ------------------------------------------------------------------
    # Freshness. Use these so a dead connection shows '--' in the GUI
    # instead of a frozen last-known value.
    # ------------------------------------------------------------------

    def _fresh(self, stamp):
        return stamp is not None and (time.monotonic() - stamp) < STALE_AFTER_SEC

    def status_is_fresh(self):
        return self._fresh(self.status_time)

    def motor_pwm_is_fresh(self):
        return self._fresh(self.motor_pwm_time)

    def pose_is_fresh(self):
        return self._fresh(self.pose_time)

    def vicon_is_fresh(self):
        return self._fresh(self.vicon_poses_time)

    # ------------------------------------------------------------------
    # Derived values. Call with parens: model.battery_voltage()
    # Each returns None when data isn't available, so the GUI can show '--'.
    # ------------------------------------------------------------------

    def battery_voltage(self):
        """Volts, or None."""
        if self.status is None:
            return None
        return self.status.battery_voltage

    def rssi(self):
        """Radio signal strength (dBm), or None. Sags before a link fully drops."""
        if self.status is None:
            return None
        return self.status.rssi

    def is_connected(self):
        """No message has a 'connected' field - the drone going quiet IS the
        signal. /status publishes at 1 Hz, so silence means the link is gone."""
        return self.status_is_fresh()

    def is_armed(self):
        if self.status is None:
            return None
        return bool(self.status.supervisor_info & Status.SUPERVISOR_INFO_IS_ARMED)

    def is_tumbled(self):
        if self.status is None:
            return None
        return bool(self.status.supervisor_info & Status.SUPERVISOR_INFO_IS_TUMBLED)

    def is_flying(self):
        if self.status is None:
            return None
        return bool(self.status.supervisor_info & Status.SUPERVISOR_INFO_IS_FLYING)

    def motors(self):
        """(m1, m2, m3, m4) PWM counts 0-65535, or None.

        The order comes from the vars: list under motor_pwm in crazyflies.yaml -
        ["motor.m1", "motor.m2", "motor.m3", "motor.m4"]. Reorder that yaml and
        this silently reorders too.
        """
        if self.motor_pwm is None or len(self.motor_pwm.values) < 4:
            return None
        v = self.motor_pwm.values
        return (v[0], v[1], v[2], v[3])

    def motors_saturated(self):
        """True if any motor is pegged at max - no attitude headroom left."""
        m = self.motors()
        if m is None:
            return None
        return max(m) >= MOTOR_MAX_PWM

    def estimated_position(self):
        """(x, y, z) the drone's own EKF believes it is at, or None."""
        if self.pose is None:
            return None
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def estimated_rpy(self):
        """(roll, pitch, yaw) degrees from the EKF quaternion, or None."""
        if self.pose is None or not _quat_is_valid(self.pose.pose.orientation):
            return None
        r, p, y = _quat_to_rpy(self.pose.pose.orientation)
        return (math.degrees(r), math.degrees(p), math.degrees(y))

    def _vicon_pose(self):
        """Our drone's entry in the NamedPoseArray, matched by name, or None.

        crazyflie_server matches on this same name, so if it doesn't match
        crazyflies.yaml the pose is silently ignored there too - a blank Vicon
        pane can mean a name mismatch rather than a mocap problem.
        """
        if self.vicon_poses is None:
            return None
        for named in self.vicon_poses.poses:
            if named.name == self.drone_name:
                return named.pose
        return None

    def vicon_position(self):
        """(x, y, z) ground truth, or None."""
        pose = self._vicon_pose()
        if pose is None:
            return None
        return (pose.position.x, pose.position.y, pose.position.z)

    def vicon_rpy(self):
        """(roll, pitch, yaw) degrees, or None.

        None in position-only mode: the bridge writes NaN into the quaternion,
        which is exactly how crazyflie_server decides to send position-only
        updates instead of full pose.
        """
        pose = self._vicon_pose()
        if pose is None or not _quat_is_valid(pose.orientation):
            return None
        r, p, y = _quat_to_rpy(pose.orientation)
        return (math.degrees(r), math.degrees(p), math.degrees(y))

    def _tf_transform(self):
        """The mocap -> <drone> transform, or None if it isn't being published."""
        try:
            return self.tf_buffer.lookup_transform(
                'mocap', self.drone_name, rclpy.time.Time()).transform
        except tf2_ros.TransformException:
            return None

    def tf_position(self):
        """(x, y, z) from TF, or None. In sim this is the only pose there is."""
        t = self._tf_transform()
        return None if t is None else (t.translation.x, t.translation.y, t.translation.z)

    def tf_rpy(self):
        """(roll, pitch, yaw) degrees from TF, or None."""
        t = self._tf_transform()
        if t is None or not _quat_is_valid(t.rotation):
            return None
        r, p, y = _quat_to_rpy(t.rotation)
        return (math.degrees(r), math.degrees(p), math.degrees(y))

    def position_error_m(self):
        """Straight-line distance between EKF estimate and Vicon, in metres."""
        est = self.estimated_position()
        vic = self.vicon_position()
        if est is None or vic is None:
            return None
        return math.dist(est, vic)

    def position_error_pct(self):
        err = self.position_error_m()
        if err is None:
            return None
        return _pct(err, POSITION_ERROR_FULL_SCALE_M)

    def attitude_error_deg(self):
        """Total angular difference between the two orientations, in degrees.
        A single number - use rpy_error_deg() to see which axis is at fault."""
        if self.pose is None:
            return None
        vicon_pose = self._vicon_pose()
        if vicon_pose is None:
            return None
        a, b = self.pose.pose.orientation, vicon_pose.orientation
        if not _quat_is_valid(a) or not _quat_is_valid(b):
            return None
        # abs() because q and -q describe the same rotation
        dot = min(1.0, abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w))
        return math.degrees(2.0 * math.acos(dot))

    def attitude_error_pct(self):
        err = self.attitude_error_deg()
        if err is None:
            return None
        return _pct(err, ATTITUDE_ERROR_FULL_SCALE_DEG)

    def rpy_error_deg(self):
        """(roll_err, pitch_err, yaw_err) degrees, each wrapped to +/-180.
        Yaw is the one that caused the runaway flights."""
        est = self.estimated_rpy()
        vic = self.vicon_rpy()
        if est is None or vic is None:
            return None
        return tuple(math.degrees(_wrap_pi(math.radians(e - v)))
                     for e, v in zip(est, vic))

    def rpy_error_pct(self):
        """0-100 driven by the WORST of the three axes, so one bad axis can't
        hide behind two good ones."""
        err = self.rpy_error_deg()
        if err is None:
            return None
        return _pct(max(abs(e) for e in err), ATTITUDE_ERROR_FULL_SCALE_DEG)

    def m1_m3_split(self):
        """SIGNED difference m1 - m3, in raw PWM counts.

        The sign is the diagnosis, not noise: positive means M1 is being driven
        harder than M3. The bent-shaft flights showed a persistent one-way split
        (+4.7k growing to +21k), which an absolute value would have hidden.
        """
        m = self.motors()
        if m is None:
            return None
        return m[0] - m[2]

    def m2_m4_split(self):
        """SIGNED difference m2 - m4, in raw PWM counts."""
        m = self.motors()
        if m is None:
            return None
        return m[1] - m[3]

    # ------------------------------------------------------------------
    # Commands out. publish() returns immediately - it never waits for the
    # controller, so a click can never freeze the window.
    # ------------------------------------------------------------------

    def _send(self, command, **fields):
        msg = DebugFlags()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = command
        msg.drone_name = self.drone_name
        for key, value in fields.items():
            setattr(msg, key, float(value))
        self.command_pub.publish(msg)
        return msg

    def send_takeoff(self, height=0.5):
        self._send(DebugFlags.CMD_TAKEOFF, height=height)

    def send_land(self):
        self._send(DebugFlags.CMD_LAND)

    def send_goto(self, x, y, z, yaw=0.0, duration=2.0):
        self._send(DebugFlags.CMD_GOTO, x=x, y=y, z=z, yaw=yaw, duration=duration)

    def send_estop(self):
        self._send(DebugFlags.CMD_ESTOP)

    # ------------------------------------------------------------------

    def all_motor_spread(self):
        """Hardest-worked minus least-worked motor, raw counts. Unsigned by
        nature - it's a range, so there is no direction to preserve."""
        m = self.motors()
        if m is None:
            return None
        return max(m) - min(m)

def main(args=None):
    """Headless run: spins the node and prints telemetry, no GUI. Useful for
    checking topics/QoS are actually connected without starting Qt."""
    import rclpy
    rclpy.init(args=args)
    node = TelemetryModel()

    def report():
        node.get_logger().info(
            f'batt={node.battery_voltage()} armed={node.is_armed()} '
            f'motors={node.motors()} pos_err={node.position_error_m()}')

    node.create_timer(1.0, report)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
