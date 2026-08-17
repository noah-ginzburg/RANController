import sys
import time
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm
from crazyflie_interfaces.msg import FullState, VelocityWorld
from crazyflie_debug_interfaces.msg import DebugFlags
from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.duration import Duration as RCLDuration
from builtin_interfaces.msg import Duration
import tf2_ros
from geometry_msgs.msg import Twist, Vector3
from rclpy.signals import SignalHandlerOptions


#Simple indexing
X_DIR = ROLL = 0
Y_DIR = PITCH = 1
Z_DIR = YAW = 2



class DroneController(Node):
    # All the tuning values used to be class constants here. They now come from
    # config/controller_params.yaml, loaded by _load_tuning_params(), which sets
    # them as instance attributes under the same names. They're still read once
    # at startup and never written again, so keeping the UPPER_CASE naming the
    # flight logic already used is deliberate rather than an oversight.

    def __init__(self):
        super().__init__('drone_controller')
        self.declare_parameter('drone_name', 'cf01')
        self.declare_parameter('hover_speed', 0.0)
        self.declare_parameter('real', True)
        # Base launch height, same for every drone.
        self.declare_parameter('launch_height', 0.5)
        # Per-drone vertical offset added on top of launch_height at takeoff --
        # e.g. the icosphere target drones (cf01/cf02/cf03) each add their
        # node's z (see delta_z in crazyflies.yaml) so the trio ends up spread
        # across the equilateral triangle instead of level with each other.
        # Non-target drones (e.g. cf09) leave this at the default 0.0.
        self.declare_parameter('delta_z', 0.0)
        # Set by real_drone_teleop.launch.py. Teleop has no takeoff button, and
        # /cmd_vel can only steer a drone that's already in the air, so when this
        # is True the controller takes off by itself at startup. Every other
        # bringup leaves it False and waits for the debug GUI instead.
        self.declare_parameter('teleop_enabled', False)

        self.drone_name = self.get_parameter('drone_name').value
        self.hover_speed = self.get_parameter('hover_speed').value
        self.real = self.get_parameter('real').value
        self.launch_height = self.get_parameter('launch_height').value
        self.delta_z = self.get_parameter('delta_z').value
        self.teleop_enabled = self.get_parameter('teleop_enabled').value

        # Must run before anything reads UPDATE_RATE (the timer below).
        self._load_tuning_params()

        self.cli = self.create_client(Takeoff, f'{self.drone_name}/takeoff')
        self.land_cli = self.create_client(Land, f'{self.drone_name}/land')
        self.arm_cli = self.create_client(Arm, f'{self.drone_name}/arm')
        # NOTE: the service is "go_to", not "goto" - both the sim
        # (crazyflie_sim/crazyflie_server.py) and the real backend
        # (crazyflie_server.cpp) advertise it with the underscore. A client on
        # the wrong name never matches and call_async silently does nothing.
        self.goto_cli = self.create_client(GoTo, f'{self.drone_name}/go_to')
        self.emergency_cli = self.create_client(Empty, f'{self.drone_name}/emergency')


        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.drone_name}/takeoff service not available, waiting again...')

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.des_heading_sub = self.create_subscription(Vector3, f'{self.drone_name}/desired_heading', self.des_heading_callback, 10)
        # Commands from the debug GUI. The GUI publishes instead of calling the
        # services itself so a slow/hung service can't freeze its window.
        self.debug_cmd_sub = self.create_subscription(
            DebugFlags, f'/{self.drone_name}/debug_command', self.debug_command_callback, 10)

        self.create_timer(1.0 / self.UPDATE_RATE, self.update)
        self.prev_time = self.get_clock().now()
        self.now = 0.0

        self.w_abs_desired = 0.0

        self.pos = np.array([0.0, 0.0, 0.0])
        self.prev_pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.prev_vel = np.array([0.0, 0.0, 0.0])
        self.acc = np.array([0.0, 0.0, 0.0])

        self.vel_desired = np.array([0.0, 0.0, 0.0])
        self.pos_desired = None
        # Active GoTo target (sim). pos_desired is walked toward this at
        # GOTO_SPEED instead of jumping, otherwise the setpoint steps and the
        # drone lunges at it.
        self.goto_target = None
        # Commanded yaw (rad). None = leave orientation as TF reports it.
        self.yaw_desired = None
        self.yaw_target = None
        # REAL only: while set, the high-level GoTo owns the drone and update()
        # publishes nothing, so our streaming setpoints can't fight it.
        self.goto_wait = None
        self.goto_wait_deadline = None

        self.movement_msg = FullState()
        self.movement_msg.acc.x = 0.0
        self.movement_msg.acc.y = 0.0
        self.movement_msg.acc.z = 0.0

        if self.real:
            self.movement_pub = self.create_publisher(VelocityWorld, f'{self.drone_name}/cmd_velocity_world', 10)
        else:
            self.movement_pub = self.create_publisher(FullState, f'{self.drone_name}/cmd_full_state', 10)

        self.should_hover = False
        self.should_land = False
        self.taking_off = True
        # Has a takeoff actually been COMMANDED? update() refuses to publish any
        # setpoint until this is True. Without it, taking_off is only ever
        # cleared by _verify_launch_completed(), which is a pure height check --
        # so picking the drone up above launch_height - 0.1 looked like a
        # finished takeoff and started the 50 Hz setpoint stream in your hands.
        # The firmware auto-arms (supervisor.c AUTO_ARMING, set unless
        # CONFIG_MOTORS_REQUIRE_ARMING), so an arm(False) at startup does NOT
        # make that safe -- this flag is the actual interlock.
        self.launch_requested = False
        self.tf_ready = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.prev_trans = None

        self.last_teleop_msg_time = self.get_clock().now()

    @staticmethod
    def _to_duration(seconds):
        """float seconds -> builtin_interfaces/Duration, for the crazyswarm2 srvs."""
        seconds = float(seconds)
        return Duration(sec=int(seconds),
                        nanosec=int(round((seconds - int(seconds)) * 1e9)))

    def _load_tuning_params(self):
        """Declare and read everything in config/controller_params.yaml.

        The defaults below are the values these were hardcoded to before the yaml
        existed, so if the params file is missing or only partly filled in, the
        controller behaves exactly as the old code did.
        """
        defaults = {
            'update_rate': 50.0,
            'max_speed': 0.2,
            'teleop_timeout': 0.5,
            'launch_duration': 5.0,
            'takeoff_max_ground_height': 0.1,
            'launch_complete_tol': 0.1,
            'launch_complete_vel': 0.1,
            'land_duration': 5.0,
            'land_height': 0.0,
            'land_min_height': 0.5,
            'goto_duration': 5.0,
            'goto_speed': 0.4,
            'goto_yaw_speed': 0.8,
            'goto_arrived_m': 0.02,
            'goto_arrived_rad': 0.02,
            'goto_reached_m': 0.05,
            'goto_timeout_s': 20.0,
            'group_mask': 0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in defaults}

        self.UPDATE_RATE = float(p['update_rate'])
        self.GROUP_MASK = int(p['group_mask'])

        self.max_speed = float(p['max_speed'])
        self.teleop_timeout = RCLDuration(seconds=float(p['teleop_timeout']))

        self.LAUNCH_DURATION = self._to_duration(p['launch_duration'])
        self.TAKEOFF_MAX_GROUND_HEIGHT = float(p['takeoff_max_ground_height'])
        self.LAUNCH_COMPLETE_TOL = float(p['launch_complete_tol'])
        self.LAUNCH_COMPLETE_VEL = float(p['launch_complete_vel'])

        # This is kept as a float as well as a Duration, because the Ctrl+C path
        # sleeps for the length of the descent. Reading Duration.sec there would
        # truncate a fractional value and wait less time than the land actually
        # takes.
        self.land_duration_s = float(p['land_duration'])
        self.LAND_DURATION = self._to_duration(self.land_duration_s)

        # A negative land height aims the descent planner below the floor, which
        # means the controller keeps pushing down into the ground instead of
        # levelling off. Refuse the value rather than fly it.
        land_height = float(p['land_height'])
        if land_height < 0.0:
            self.get_logger().error(
                f'land_height is {land_height} m, which aims the landing planner '
                'below the floor. Clamping to 0.0 -- fix controller_params.yaml.')
            land_height = 0.0
        self.LAND_HEIGHT = land_height
        self.LAND_MIN_HEIGHT = float(p['land_min_height'])

        self.GOTO_DURATION = self._to_duration(p['goto_duration'])
        self.GOTO_SPEED = float(p['goto_speed'])
        self.GOTO_YAW_SPEED = float(p['goto_yaw_speed'])
        self.GOTO_ARRIVED_M = float(p['goto_arrived_m'])
        self.GOTO_ARRIVED_RAD = float(p['goto_arrived_rad'])
        self.GOTO_REACHED_M = float(p['goto_reached_m'])
        self.GOTO_TIMEOUT_S = float(p['goto_timeout_s'])

    def send_arm_req(self, arm: bool):
        # Only the cflib/cpp (real) backends provide an arm service; the sim backend
        # does not. Without this guard the call_async future never completes and the
        # node hangs forever before takeoff. Bail if no server is present, and bound
        # the spin so a silent server can't wedge us either.
        if not self.arm_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'{self.drone_name}/arm service unavailable; skipping arm (expected in sim).')
            return None

        req = Arm.Request()
        req.arm = arm

        self.future = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5.0)
        return self.future.result()

    def send_takeoff_req(self, group_mask, height, duration):
        self.launch_requested = True
        req = Takeoff.Request()
        req.group_mask = group_mask
        req.height = height
        req.duration = duration

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_land_req(self, group_mask, height, duration):
        req = Land.Request()
        req.group_mask = group_mask
        req.height = height
        req.duration = duration

        self.future = self.land_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_emergency_req(self):
        # Cuts the motors immediate ly. Bounded spin so a wedged executor can't
        # block the panic stop, but we still pump the link once to transmit it.
        future = self.emergency_cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result()

    def send_goto_req(self, group_mask, relative, goal, yaw, duration):

        req = GoTo.Request()
        req.group_mask = group_mask
        req.relative = relative
        req.goal = goal
        req.yaw = yaw
        req.duration = duration

        self.future = self.goto_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    # TAKEOFF_MAX_GROUND_HEIGHT, GOTO_SPEED, GOTO_ARRIVED_M, GOTO_YAW_SPEED,
    # GOTO_ARRIVED_RAD, GOTO_REACHED_M and GOTO_TIMEOUT_S all live in
    # config/controller_params.yaml now. _load_tuning_params() reads them.

    def _await_goto(self, now):
        """REAL only. True means 'still travelling - update() must not publish'.

        While the high-level GoTo runs, our own 50 Hz setpoint stream would
        fight it, so we stay silent until the drone arrives. Bounded by a
        timeout: a drone that never reaches the target must not wedge the
        controller forever with no way to command it again.
        """
        if self.goto_wait is None:
            return False

        if self.tf_ready and not np.isnan(self.pos).any():
            dist = float(np.linalg.norm(self.pos - self.goto_wait))
            if dist <= self.GOTO_REACHED_M:
                self.get_logger().info(
                    f'GoTo: reached target (within {dist:.3f} m); resuming control.')
                # Bookkeeping only on the real path: nothing consumes
                # pos_desired there (see _record_hover_setpoint). What actually
                # holds the drone at the target after a real GoTo is the
                # onboard high-level commander, not this value.
                self.pos_desired = self.goto_wait.copy()
                self.goto_wait = None
                self.goto_wait_deadline = None
                return False

        if now > self.goto_wait_deadline:
            self.get_logger().error(
                f'GoTo: TIMED OUT after {self.GOTO_TIMEOUT_S:.0f}s without reaching '
                f'{self.goto_wait}; resuming control so the drone stays commandable.')
            self.pos_desired = None
            self.goto_wait = None
            self.goto_wait_deadline = None
            return False

        return True

    @staticmethod
    def _yaw_from_quat(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return float(np.arctan2(siny, cosy))

    def _step_yaw(self, dt):
        """Walk yaw_desired toward yaw_target, then force it into the command.

        Without this, _update_pos() copies the MEASURED orientation into the
        command every cycle, so the commanded yaw always equals the current
        yaw and the drone can never rotate.
        """
        if self.yaw_desired is None:
            return
        if self.yaw_target is not None and dt > 0.0:
            dt = min(dt, 2.0 / self.UPDATE_RATE)
            err = (self.yaw_target - self.yaw_desired + np.pi) % (2 * np.pi) - np.pi
            step = self.GOTO_YAW_SPEED * dt
            if abs(err) <= max(step, self.GOTO_ARRIVED_RAD):
                self.yaw_desired = self.yaw_target
                self.yaw_target = None
                self.get_logger().info(
                    f'GoTo: yaw arrived at {np.degrees(self.yaw_desired):.1f} deg')
            else:
                self.yaw_desired += np.sign(err) * step

        # Overwrite the orientation _update_pos() just copied from TF.
        self.movement_msg.pose.orientation.x = 0.0
        self.movement_msg.pose.orientation.y = 0.0
        self.movement_msg.pose.orientation.z = float(np.sin(self.yaw_desired / 2.0))
        self.movement_msg.pose.orientation.w = float(np.cos(self.yaw_desired / 2.0))

    def _record_hover_setpoint(self):
        """SIM ONLY. Latch the current position as the hover setpoint.

        Gated to sim because pos_desired only means something there. On the
        real drone update() publishes VelocityWorld, which puts the firmware in
        modeVelocity -- and position_controller_pid.c runs its position PID
        only `if (setpoint->mode.x == modeAbs)`, so the onboard position loop
        is bypassed entirely and no position setpoint we compute here can reach
        it. Latching one on the real path was pure bookkeeping: it read like a
        hover was being commanded when the drone was actually being told
        "hold zero velocity", which has no position feedback at all and lets
        the drone integrate away (bags/flight_20260813_143119 and _143207:
        divergence starts within 200 ms of this stream taking over).
        """
        if self.real:
            return
        if self.pos_desired is None and not self.taking_off:
            self.pos_desired = self.pos.copy()
            self.get_logger().warn(
                f'drone {self.drone_name} hovering at {self.pos_desired}.')

    def _step_goto(self, dt):
        """Walk pos_desired toward goto_target at GOTO_SPEED."""
        if self.goto_target is None or dt <= 0.0:
            return
        if self.pos_desired is None:
            self.pos_desired = self.pos.copy()
        # prev_time is only updated at the END of update(), and several paths
        # return early, so dt can arrive very large. Uncapped, that turns one
        # tick into a jump straight to the target - the lunge we are avoiding.
        dt = min(dt, 1.0 / self.UPDATE_RATE * 2.0)

        delta = self.goto_target - self.pos_desired
        dist = float(np.linalg.norm(delta))
        step = self.GOTO_SPEED * dt
        if dist <= max(step, self.GOTO_ARRIVED_M):
            self.pos_desired = self.goto_target.copy()
            self.goto_target = None
            self.get_logger().info(f'GoTo: setpoint arrived at {self.pos_desired}')
        else:
            self.pos_desired = self.pos_desired + (delta / dist) * step

    def debug_command_callback(self, msg: DebugFlags):
        """Handle one command from the debug GUI.

        Everything here uses call_async, NOT send_*_req(): those spin the node
        with spin_until_future_complete, and doing that inside a callback
        re-enters the executor we are already spinning in.
        """
        if msg.drone_name and msg.drone_name != self.drone_name:
            return

        if msg.command == DebugFlags.CMD_TAKEOFF:
            self._debug_takeoff(msg)
        elif msg.command == DebugFlags.CMD_LAND:
            self._debug_land()
        elif msg.command == DebugFlags.CMD_ESTOP:
            self.get_logger().fatal(
                f'EMERGENCY STOP from debug GUI - cutting motors on {self.drone_name}.')
            self.should_land = True
            self.taking_off = False
            self.launch_requested = False
            self.goto_target = None
            self.goto_wait = None
            self.emergency_cli.call_async(Empty.Request())
        elif msg.command == DebugFlags.CMD_GOTO:
            self._debug_goto(msg)

    def _debug_takeoff(self, msg: DebugFlags):
        # No TF means we cannot prove the drone is on the ground, so we refuse
        # rather than assume. Silence is not evidence of being landed.
        if not self.tf_ready:
            self.get_logger().error(
                'Takeoff REFUSED: no TF yet, cannot verify the drone is on the ground.')
            return

        z = float(self.pos[Z_DIR])
        if np.isnan(z):
            self.get_logger().error('Takeoff REFUSED: current z is NaN (tracking lost).')
            return
        if z > self.TAKEOFF_MAX_GROUND_HEIGHT:
            self.get_logger().error(
                f'Takeoff REFUSED: drone is at z={z:.3f} m, above the '
                f'{self.TAKEOFF_MAX_GROUND_HEIGHT:.2f} m ground limit. '
                'It is already airborne or being held.')
            return

        height = msg.height if msg.height > 0.0 else (self.launch_height + self.delta_z)
        self.get_logger().info(
            f'Takeoff from debug GUI: z={z:.3f} m -> {height:.3f} m')

        self.taking_off = True
        self.launch_requested = True
        self.should_land = False
        self.pos_desired = None

        req = Takeoff.Request()
        req.group_mask = self.GROUP_MASK
        req.height = float(height)
        req.duration = self.LAUNCH_DURATION
        self.cli.call_async(req)

    def _debug_land(self):
        self.get_logger().info(f'Land from debug GUI ({self.drone_name}).')
        self.should_land = True
        self.taking_off = False
        # Back on the ground (or on the way there): a new takeoff must be
        # commanded before anything may stream setpoints again.
        self.launch_requested = False
        self.goto_target = None      # abandon any in-progress GoTo
        self.goto_wait = None

        # Land.height is an ABSOLUTE target altitude, not a descent distance:
        # crazyswarm2 hands it straight to cflib's
        # high_level_commander.land(absolute_height_m, ...). The old
        # -self.pos[Z_DIR] therefore aimed the planner BELOW the floor (at z=1.2
        # it commanded -1.2 m), so the descent ramp never levelled off at the
        # ground -- it drove the setpoint underground and the controller kept
        # pushing down into the floor. Ground is 0.0, full stop; this also drops
        # the old NaN/stale-TF hazard, since the value no longer depends on pos.
        req = Land.Request()
        req.group_mask = self.GROUP_MASK
        req.height = self.LAND_HEIGHT
        req.duration = self.LAND_DURATION
        self.land_cli.call_async(req)

    def _debug_goto(self, msg: DebugFlags):
        self.taking_off = False
        self.should_land = False
        # An explicit GoTo is a flight command like takeoff, so it authorises the
        # setpoint stream too -- otherwise update() would go quiet on arrival and
        # leave the drone parked on the high-level commander.
        self.launch_requested = True

        if not self.real:
            # SIM: the high-level go_to service is unusable here. Streaming
            # cmd_full_state puts the sim in MODE_LOW_FULLSTATE, and
            # crazyflie_sil.goTo() only accepts MODE_HIGH_POLY - it raises
            # "goTo from low-level modes not yet supported", which kills the
            # service callback (that is why a second click then reports the
            # service as missing). Only takeoff/land restore HIGH_POLY, so the
            # service can never work once our control loop is running.
            # Our own loop already position-controls in sim, so move the
            # setpoint instead. NOTE: yaw is not applied on this path.
            self.should_hover = True
            if self.pos_desired is None:
                self.pos_desired = self.pos.copy()
            # Ramp toward it in update(); assigning it directly is a step change
            # and the drone lunges.
            self.goto_target = np.array([msg.x, msg.y, msg.z], dtype=float)
            # Start the yaw ramp from where the drone actually is, otherwise the
            # first step is a jump from 0.
            if self.yaw_desired is None:
                self.yaw_desired = self._yaw_from_quat(
                    self.movement_msg.pose.orientation)
            self.yaw_target = float(msg.yaw)
            self.get_logger().info(
                f'GoTo (sim setpoint): [{msg.x:.3f} {msg.y:.3f} {msg.z:.3f}] '
                f'yaw={np.degrees(msg.yaw):.1f} deg, ramping at '
                f'{self.GOTO_SPEED} m/s / {self.GOTO_YAW_SPEED} rad/s')
            return

        if not self.goto_cli.service_is_ready():
            self.get_logger().error(
                f'GoTo REFUSED: {self.drone_name}/go_to service not available.')
            return
        self.get_logger().info(
            f'GoTo from debug GUI: [{msg.x:.3f} {msg.y:.3f} {msg.z:.3f}] yaw={msg.yaw:.3f}')
        self.should_hover = False

        req = GoTo.Request()
        req.group_mask = self.GROUP_MASK
        req.relative = False
        req.goal.x = float(msg.x)
        req.goal.y = float(msg.y)
        req.goal.z = float(msg.z)
        req.yaw = float(msg.yaw)
        req.duration = Duration(sec=int(msg.duration), nanosec=0) if msg.duration > 0 else self.GOTO_DURATION

        # call_async, never spin_until_future_complete: we are inside a
        # subscription callback and spinning here re-enters the executor.
        self.goto_cli.call_async(req)

        # Hand the drone to the high-level commander and go quiet until it
        # arrives at the position from this very message.
        self.goto_wait = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.goto_wait_deadline = self.get_clock().now() + RCLDuration(
            seconds=self.GOTO_TIMEOUT_S)
        self.pos_desired = None
        self.get_logger().info(
            f'GoTo: holding all other control until {self.goto_wait} is reached '
            f'(timeout {self.GOTO_TIMEOUT_S:.0f}s).')

    def cmd_vel_callback(self, msg: Twist):
        self.taking_off = False
        self.should_hover = False
        self.last_teleop_msg_time = self.get_clock().now()
        self.get_logger().info(f'Teleop: setting speeds to {[msg.linear.x, msg.linear.y, msg.linear.z], [msg.angular.x, msg.angular.y, msg.angular.z]}')
        self.set_speeds([msg.linear.x, msg.linear.y, msg.linear.z], [msg.angular.x, msg.angular.y, msg.angular.z])


    def _snap_vector(self, v):
        v[np.abs(v) < self.SNAP_THRESHOLD] = 0.0
        norm = np.linalg.norm(v)
        return v / norm if norm > 0 else v

    def des_heading_callback(self, msg: Vector3):
        self.should_hover = False
        self.vel_desired = self.max_speed * np.array([msg.x, msg.y, msg.z])

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        time_from_teleop = now - self.last_teleop_msg_time
        # self.get_logger().info(f'last time from teleop: {time_from_teleop}')
        
        # self.get_logger().info("logging")
        self._update_pos(dt)

        # REAL: while a high-level GoTo is travelling, publish nothing at all -
        # our stream would override it. Position tracking above still runs so we
        # can see when we arrive. prev_time/prev_pos are advanced before the
        # early return, otherwise dt accumulates and the next tick jumps.
        if self._await_goto(now):
            self.prev_time = now
            self.prev_pos = self.pos.copy()
            return

        self._step_goto(dt)
        self._step_yaw(dt)      # must run AFTER _update_pos, which resets orientation

        commanded_vel = np.array(self.vel_desired, dtype=float)

        if time_from_teleop > self.teleop_timeout:
            if self.should_hover:
                # np.array(0.0, 0.0, 0.0) was a TypeError waiting to happen -
                # numpy reads the 2nd positional as dtype. should_hover is set
                # from a TF dropout, so the first tracking hiccup would have
                # raised inside the timer callback and taken the node down.
                commanded_vel = np.zeros(3)
                self._record_hover_setpoint()

            self.set_speeds(commanded_vel)
        else:
            # Teleop has taken over - drop any in-progress GoTo with it.
            self.pos_desired = None
            self.goto_target = None
            self.goto_wait = None
            # self.get_logger().info(f'wrong area')

        if not self.tf_ready:
            self.get_logger().warn("No tf ready, exiting update loop")
            self.prev_time = now
            self.prev_pos = self.pos.copy()
            return

        # Hard interlock: no commanded takeoff, no setpoints. Handling the drone
        # must never be enough to start the stream.
        if not self.launch_requested:
            self.prev_time = now
            self.prev_pos = self.pos.copy()
            return

        if self.taking_off:
            if self._verify_launch_completed():
                self.taking_off = False
                if not self.real:
                    # SIM ONLY - see _record_hover_setpoint(). On the real path
                    # this setpoint had no consumer: the VelocityWorld stream
                    # below cannot express it.
                    self.pos_desired = self.pos.copy()
                    self.pos_desired[Z_DIR] = self.launch_height + self.delta_z
            else:
                # prev_* must advance here too, otherwise dt grows without bound
                # while we climb and self.vel = (pos - prev_pos)/dt collapses to
                # ~0 -- which silently defeated the vel[Z] <= 0.1 half of
                # _verify_launch_completed().
                self.prev_time = now
                self.prev_pos = self.pos.copy()
                return
        if self.should_land:
            self.prev_time = now
            self.prev_pos = self.pos.copy()
            return

        self.movement_msg.header.stamp = self.get_clock().now().to_msg()
        self.movement_msg.header.frame_id = self.drone_name

        if self.real:
            velocity_msg = VelocityWorld()
            velocity_msg.header = self.movement_msg.header
            velocity_msg.vel.x = self.movement_msg.twist.linear.x
            velocity_msg.vel.y = self.movement_msg.twist.linear.y
            velocity_msg.vel.z = self.movement_msg.twist.linear.z
            velocity_msg.yaw_rate = self.movement_msg.twist.angular.z
            self.movement_pub.publish(velocity_msg)
        else:
            self.movement_pub.publish(self.movement_msg)
            # self.get_logger().info(f'sim: speeds = {self.movement_msg.twist.linear.x}, {self.movement_msg.twist.linear.y}, {self.movement_msg.twist.linear.z}')

        self.prev_time = now
        self.prev_pos = self.pos.copy()

    def set_speeds(self, lin_speeds, ang_speeds=None):
        self.movement_msg.twist.linear.x = lin_speeds[X_DIR]
        self.movement_msg.twist.linear.y = lin_speeds[Y_DIR]
        hover_bias = self.hover_speed if not self.real else 0.0
        self.movement_msg.twist.linear.z = lin_speeds[Z_DIR] + hover_bias

        if not ang_speeds == None:
            self.movement_msg.twist.angular.x = ang_speeds[ROLL]
            self.movement_msg.twist.angular.y = ang_speeds[PITCH]
            self.movement_msg.twist.angular.z = ang_speeds[YAW]

        # self.get_logger().info(f'settings speeds to {self.movement_msg.twist}')

    def _verify_tf_available(self):
        if not self.tf_buffer.can_transform('mocap', self.drone_name, rclpy.time.Time()): 
            return False
        return True

    def _retrieve_tf_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform('mocap', f'{self.drone_name}', rclpy.time.Time(), RCLDuration(seconds=1.0))
            return trans
        except (tf2_ros.TransformException) as e:
            self.get_logger().warn(str(e) + ", drone is being forced to hover")
            self.should_hover = True
            return None


    def _update_pos(self, dt):
        if not self._verify_tf_available(): return

        trans = self._retrieve_tf_transform()
        if not trans == None:
            self.movement_msg.pose.position.x = self.pos[X_DIR] = trans.transform.translation.x
            self.movement_msg.pose.position.y = self.pos[Y_DIR] = trans.transform.translation.y
            self.movement_msg.pose.position.z = self.pos[Z_DIR] = trans.transform.translation.z
            self.movement_msg.pose.orientation = trans.transform.rotation

            # self.get_logger().info(f'current z_pos from tf: {trans.transform.translation.z}')
            # self.get_logger().info(f'current vel from prev update: {self.vel[Z_DIR]}')
            self.vel = (self.pos - self.prev_pos)/dt
            self.tf_ready = True
            self.prev_trans = trans
    
    def _verify_launch_completed(self):
        if not self._verify_tf_available(): return
        
        trans = self._retrieve_tf_transform()
        if not trans == None:
        
            #debugging
            # if self.drone_name == 'cf09':
            #     self.get_logger().info(f"z_pos: {self.pos[Z_DIR]}")
            #     self.get_logger().info(f"launch height: {self.launch_height + self.delta_z}")

            if (self.pos[Z_DIR] >= (self.launch_height + self.delta_z - self.LAUNCH_COMPLETE_TOL)
                    and self.vel[Z_DIR] <= self.LAUNCH_COMPLETE_VEL):
                self.get_logger().info(f'Drone {self.drone_name} completed launch, current height: {self.pos[Z_DIR]}')
                return True
    
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)    
    # rclpy.init(args=args)    
    drone_controller = DroneController()

    if drone_controller.real:
        arm_response = drone_controller.send_arm_req(True)

        if arm_response is not None:
            drone_controller.get_logger().info('Arm command executed successfully')
        else:
            # None means no arm server responded — normal in sim, already warned above.
            drone_controller.get_logger().info('Arm skipped (no arm service); continuing to takeoff.')

    if drone_controller.teleop_enabled:
        # Teleop has no takeoff button, and /cmd_vel can only steer a drone
        # that's already in the air, so take off here. send_takeoff_req is also
        # what sets launch_requested, which is the interlock that authorises the
        # 50 Hz setpoint stream. Without a takeoff the teleop keys do nothing at
        # all.
        target_height = drone_controller.launch_height + drone_controller.delta_z
        drone_controller.get_logger().info(
            f'teleop_enabled: auto-taking off {drone_controller.drone_name} to '
            f'{target_height:.2f} m. Do not touch the keyboard until the climb '
            'finishes -- a /cmd_vel message clears taking_off and starts the '
            'setpoint stream against the high-level commander.')
        response = drone_controller.send_takeoff_req(
            group_mask=drone_controller.GROUP_MASK,
            height=target_height,
            duration=drone_controller.LAUNCH_DURATION)

        if response is not None:
            drone_controller.get_logger().info('Auto-takeoff command accepted.')
        else:
            drone_controller.get_logger().error(
                'Auto-takeoff service call returned no response; the drone is '
                'probably still on the ground. Check crazyflie_server.')
    else:
        # This isn't a failure, it's the normal path. The debug GUI commands the
        # takeoff, and that's what sets launch_requested.
        drone_controller.get_logger().info(
            f'Auto-takeoff disabled for {drone_controller.drone_name} '
            '(teleop_enabled=false). Waiting for a takeoff command from the '
            'debug GUI.')

    drone_controller.get_logger().info('Spinning.')

    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        # First Ctrl+C: land gracefully (NO emergency stop here — that cuts motors
        # and drops the drone). Emergency is reserved for a second Ctrl+C below.
        drone_controller.get_logger().info(f'User hit Ctrl+C. Attempting to land drone {drone_controller.drone_name}.')
        drone_controller.should_land = True
        drone_controller.launch_requested = False
        try:
            # Land-vs-ESTOP is decided on the drone's ACTUAL altitude. This used
            # to test abs(-pos[Z_DIR]), i.e. the land target, which no longer
            # carries any altitude information now that the target is a fixed
            # absolute 0.0 -- so the test has to read z directly.
            #
            # A stale/NaN z is treated as NOT airborne: silence is not evidence
            # of being in the air, and a land we cannot justify spins the motors.
            z = float(drone_controller.pos[Z_DIR])
            airborne = (drone_controller.tf_ready and not np.isnan(z)
                        and z > drone_controller.LAND_MIN_HEIGHT)

            if airborne:
                # height is ABSOLUTE (LAND_HEIGHT == floor), not a descent distance.
                land_height = drone_controller.LAND_HEIGHT
                land_resp = drone_controller.send_land_req(group_mask=drone_controller.GROUP_MASK, height=land_height, duration=drone_controller.LAND_DURATION)
                drone_controller.get_logger().info(f'Requesting drone {drone_controller.drone_name} to land from z={z:.3f} m to height={land_height}.')
                if land_resp is not None: time.sleep(drone_controller.land_duration_s)
            else:
                # NOTE: this branch previously called self.get_logger() -- but
                # there is no `self` in main(), so it raised NameError before
                # ever reaching send_emergency_req(). The ESTOP never fired, and
                # the NameError escaped main() entirely (the enclosing except
                # only catches KeyboardInterrupt), skipping destroy_node() and
                # rclpy.shutdown() and leaving the drone armed.
                drone_controller.get_logger().info(
                    f'Drone {drone_controller.drone_name} at z={z:.3f} m '
                    f'(tf_ready={drone_controller.tf_ready}) is below the '
                    f'{drone_controller.LAND_MIN_HEIGHT:.2f} m land threshold; requesting ESTOP.')
                try:
                    drone_controller.send_emergency_req()
                except Exception as e:
                    drone_controller.get_logger().error(f'Emergency stop call failed: {e}')
        except KeyboardInterrupt:
            drone_controller.get_logger().fatal(
                f'User hit Ctrl+C again. EMERGENCY STOP — cutting motors on {drone_controller.drone_name}.')
            try:
                drone_controller.send_emergency_req()
            except Exception as e:
                drone_controller.get_logger().error(f'Emergency stop call failed: {e}')
            pass
        pass

    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()