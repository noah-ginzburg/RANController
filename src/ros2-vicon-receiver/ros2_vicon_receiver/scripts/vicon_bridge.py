#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ros2_vicon_receiver.msg import Position
import tf2_ros
from geometry_msgs.msg import TransformStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray, NamedPose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

MARKER_HEIGHT_OFFSET = 0.0

class ViconBridge(Node):
    def __init__(self):
        super().__init__('vicon_bridge')
        self.declare_parameter('all_drones', [''])
        self.all_drones = self.get_parameter('all_drones').value
        # send_orientation=False publishes NaN quaternions in /poses, which makes
        # crazyswarm2 fall back to send_extpos (position-only fusion). This
        # sidesteps the old-firmware bug where the EKF's external-attitude update
        # diverges at yaw far from 0 (exponential roll/pitch runaway + reset loop).
        # Cost: EKF yaw is gyro-only -- POWER THE DRONE ON FACING VICON +X (EKF
        # boots believing yaw=0) and expect slow yaw drift; avoid yaw maneuvers.
        # Set to True once firmware is upgraded past the attitude-update fix.
        self.declare_parameter('send_orientation', False)
        self.send_orientation = self.get_parameter('send_orientation').value

        topics = self.get_topic_names_and_types()
        existing = {topic for topic, _ in topics}
        self.subscribers = {}
        # Last raw pose per subject, used to drop frozen frames. When Tracker
        # loses the object it re-serves the last pose bit-identically (the
        # receiver never checks the SDK's Occluded flag); real tracking always
        # has sub-mm noise between frames, so exact repeats mean stale data.
        self.last_raw = {}


        for name in self.all_drones:
            matches = [t for t in existing if t.startswith(f'/vicon/{name}/')]
            if matches:
                self.subscribers[name] = self.create_subscription(Position, f'/vicon/{name}/{name}', self.vicon_callback, 10)
                self.get_logger().info(f'topic of {name} found.')


        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        poses_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            deadline=Duration(seconds=0, nanoseconds=int(1e9 / 100.0)),
        )
        self.poses_pub = self.create_publisher(NamedPoseArray, '/poses', poses_qos)

        # Staleness watchdog: the upstream receiver goes silent (no message at
        # all) when tracking is lost or the network stalls, so gaps can't be
        # detected in the subscriber callback -- only by watching the clock.
        self.last_valid = {}  # subject_name -> rclpy Time of last forwarded pose
        self.stale_after = 0.25  # seconds without a valid frame = flying blind
        self.create_timer(0.1, self.check_staleness)

        # Boot-heading guard (position-only mode): with NaN orientation the EKF
        # never learns yaw -- it stays at whatever heading the drone had when the
        # estimator was reset (i.e. at connect). If that heading is far from
        # Vicon +X, position feedback is applied through a wrong yaw and the
        # drone runs away on takeoff (mirrored control). Nag until fixed.
        self.max_boot_yaw_deg = 15.0
        self.last_yaw = {}  # subject_name -> latest vicon yaw in degrees

    def check_staleness(self):
        now = self.get_clock().now()
        for name, t in self.last_valid.items():
            age = (now - t).nanoseconds / 1e9
            if age > self.stale_after:
                self.get_logger().warn(
                    f'{name}: no valid Vicon pose for {age:.2f}s -- flying blind',
                    throttle_duration_sec=1.0)
        if not self.send_orientation:
            for name, yaw in self.last_yaw.items():
                if abs(yaw) > self.max_boot_yaw_deg:
                    self.get_logger().error(
                        f'{name}: facing {yaw:+.0f} deg from Vicon +X. Position-only '
                        f'fusion assumes yaw=0 at estimator reset -- flying like this '
                        f'mirrors position control and the drone will run away. '
                        f'Rotate the drone to face +X, then restart the launch.',
                        throttle_duration_sec=5.0)


    def vicon_callback(self, msg: Position):
        # self.get_logger().info(f'{msg.subject_name} data received')

        # Drop occluded / invalid frames. When Vicon loses the rigid body it
        # reports translation (0,0,0) and a NaN/zero quaternion. Forwarding that
        # teleports the drone's estimator to the origin and it crashes. A brief
        # gap is fine -- the onboard Kalman filter coasts on gyro/accel.
        quat = (msg.x_rot, msg.y_rot, msg.z_rot, msg.w)
        if any(math.isnan(v) for v in quat):
            self.get_logger().warn("nan observed in quaternion")
            return
        if abs(math.sqrt(sum(v * v for v in quat)) - 1.0) > 0.1:  # not a unit quaternion
            self.get_logger().warn("not a unit quaternion")
            return
        if msg.x_trans == 0.0 and msg.y_trans == 0.0 and msg.z_trans == 0.0:
            self.get_logger().warn("invalid 0,0,0 frame")
            return

        raw = (msg.x_trans, msg.y_trans, msg.z_trans,
               msg.x_rot, msg.y_rot, msg.z_rot, msg.w)
        if self.last_raw.get(msg.subject_name) == raw:
            self.get_logger().warn(
                f'{msg.subject_name}: frozen Vicon frame (tracking lost?), dropping',
                throttle_duration_sec=1.0)
            return
        self.last_raw[msg.subject_name] = raw

        x_m = msg.x_trans / 1000.0
        y_m = msg.y_trans / 1000.0
        z_m = (msg.z_trans - MARKER_HEIGHT_OFFSET) / 1000.0

        # Normalize quaternion sign: q and -q are the same rotation, but the
        # Crazyflie EKF keeps w >= 0 internally. Vicon emits w < 0 for some
        # orientations (e.g. yaw near 180 deg); feeding the antipodal quaternion
        # makes the attitude innovation read as ~360 deg -> EKF diverges and
        # auto-resets in a loop while the drone sits still.
        qx, qy, qz, qw = msg.x_rot, msg.y_rot, msg.z_rot, msg.w
        if qw < 0.0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw

        self.last_yaw[msg.subject_name] = math.degrees(
            math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'mocap'
        transform.child_frame_id = msg.subject_name
        transform.transform.translation.x = x_m
        transform.transform.translation.y = y_m
        transform.transform.translation.z = z_m

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        named_pose = NamedPose()
        named_pose.name = msg.subject_name
        named_pose.pose.position.x = x_m
        named_pose.pose.position.y = y_m
        named_pose.pose.position.z = z_m
        if self.send_orientation:
            named_pose.pose.orientation.x = qx
            named_pose.pose.orientation.y = qy
            named_pose.pose.orientation.z = qz
            named_pose.pose.orientation.w = qw
        else:
            # NaN x makes crazyflie_server use send_extpos (position only);
            # the drone's buggy external-attitude update then never runs.
            named_pose.pose.orientation.x = float('nan')
            named_pose.pose.orientation.y = float('nan')
            named_pose.pose.orientation.z = float('nan')
            named_pose.pose.orientation.w = float('nan')

        self.poses_pub.publish(NamedPoseArray(poses=[named_pose]))
        self.last_valid[msg.subject_name] = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    vicon_bridge = ViconBridge()
    rclpy.spin(vicon_bridge)
    vicon_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
