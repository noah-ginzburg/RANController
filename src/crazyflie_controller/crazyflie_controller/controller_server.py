import sys
import time
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm
from crazyflie_interfaces.msg import FullState, VelocityWorld
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
    UPDATE_RATE = 50.0  #hz
    GROUP_MASK = 0  #0 = all drones
    DURATION = Duration(sec=2, nanosec=0)   #Time to reach the desired height


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

        self.drone_name = self.get_parameter('drone_name').value
        self.hover_speed = self.get_parameter('hover_speed').value
        self.real = self.get_parameter('real').value
        self.launch_height = self.get_parameter('launch_height').value
        self.delta_z = self.get_parameter('delta_z').value

        self.cli = self.create_client(Takeoff, f'{self.drone_name}/takeoff')
        self.land_cli = self.create_client(Land, f'{self.drone_name}/land')
        self.arm_cli = self.create_client(Arm, f'{self.drone_name}/arm')
        self.goto_cli = self.create_client(GoTo, f'{self.drone_name}/arm')
        self.emergency_cli = self.create_client(Empty, f'{self.drone_name}/emergency')


        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.drone_name}/takeoff service not available, waiting again...')

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.des_heading_sub = self.create_subscription(Vector3, f'{self.drone_name}/desired_heading', self.des_heading_callback, 10)

        self.create_timer(1.0 / self.UPDATE_RATE, self.update)
        self.prev_time = self.get_clock().now()
        self.now = 0.0

        self.w_abs_desired = 0.0
        self.max_speed = 0.5

        self.pos = np.array([0.0, 0.0, 0.0])
        self.prev_pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.prev_vel = np.array([0.0, 0.0, 0.0])
        self.acc = np.array([0.0, 0.0, 0.0])

        self.vel_desired = np.array([0.0, 0.0, 0.0])
        self.pos_desired = None

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
        self.tf_ready = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.prev_trans = None

        self.last_teleop_msg_time = self.get_clock().now()
        self.teleop_timeout = RCLDuration(seconds=0.5)

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

        self.future = self.goto_cli_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    def cmd_vel_callback(self, msg: Twist):
        self.taking_off = False
        self.should_hover = False
        self.last_teleop_msg_time = self.get_clock().now()
        self.set_speeds([msg.linear.x, msg.linear.y, msg.linear.z], [msg.angular.x, msg.angular.y, msg.angular.z])

    SNAP_THRESHOLD = 0.05

    def _snap_vector(self, v):
        v[np.abs(v) < self.SNAP_THRESHOLD] = 0.0
        norm = np.linalg.norm(v)
        return v / norm if norm > 0 else v

    def des_heading_callback(self, msg: Vector3):
        self.should_hover = False
        self.last_teleop_msg_time = self.get_clock().now()

        self.vel_desired = self.max_speed * np.array([msg.x, msg.y, msg.z])
        # if norm > 0:
        #     v = self._snap_vector(v / norm)

        ####EDIT TOMORROW
        # self.set_speeds(self.max_speed * v)

        # self.get_logger().info(f'setting speeds to {self.max_speed * v}')


    def update(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        time_from_teleop = now - self.last_teleop_msg_time
        
        # self.get_logger().info("logging")
        self._update_pos(dt)

        if time_from_teleop > self.teleop_timeout or self.should_hover:
            if self.pos_desired is None and not self.taking_off:
                self.get_logger().warn(f'recording current pos: {self.pos}')
                self.pos_desired = self.pos.copy()
            self.get_logger().warn(f'drone {self.drone_name} hovering.')
            test=1
        else:
            self.pos_desired = None
        
        if not self.tf_ready:
            self.get_logger().warn("No tf ready, exiting update loop")
            return

        if self.taking_off:
            if self._verify_launch_completed():
                self.taking_off = False
                self.pos_desired = self.pos.copy()
                self.pos_desired[Z_DIR] = self.launch_height + self.delta_z
            else:
                return
        if self.should_land: return

        commanded_vel = np.array(self.vel_desired, dtype=float)
        if not self.real and self.pos_desired is not None:
            commanded_vel = np.array([0.0, 0.0, 0.0])
            self.movement_msg.pose.position.x = self.pos_desired[X_DIR]
            self.movement_msg.pose.position.y = self.pos_desired[Y_DIR]
            self.movement_msg.pose.position.z = self.pos_desired[Z_DIR]

        self.set_speeds(commanded_vel)

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

        self.prev_time = now
        self.prev_pos = self.pos.copy()

    def set_speeds(self, lin_speeds, ang_speeds=None):
        self.movement_msg.twist.linear.x = lin_speeds[X_DIR]
        self.movement_msg.twist.linear.y = lin_speeds[Y_DIR]
        hover_bias = self.hover_speed if self.real else 0.0
        self.movement_msg.twist.linear.z = lin_speeds[Z_DIR] + hover_bias

        if not ang_speeds == None:
            self.movement_msg.twist.angular.x = ang_speeds[ROLL]
            self.movement_msg.twist.angular.y = ang_speeds[PITCH]
            self.movement_msg.twist.angular.z = ang_speeds[YAW]

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

            self.get_logger().info(f'current z_pos from tf: {trans.transform.translation.z}')
            self.get_logger().info(f'current vel from prev update: {self.vel[Z_DIR]}')
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

            if self.pos[Z_DIR] >= (self.launch_height + self.delta_z - 0.1) and self.vel[Z_DIR] <= 0.1:
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

    target_height = drone_controller.launch_height + drone_controller.delta_z
    response = drone_controller.send_takeoff_req(group_mask=drone_controller.GROUP_MASK, height=target_height, duration=drone_controller.DURATION)
    # response = None

    drone_controller.get_logger().info("got to takeoff")
    if response is not None:
        drone_controller.get_logger().info('Takeoff command executed successfully')
    else:
        drone_controller.get_logger().error('Takeoff service call failed')

    drone_controller.get_logger().info("trying to spin")

    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        # First Ctrl+C: land gracefully (NO emergency stop here — that cuts motors
        # and drops the drone). Emergency is reserved for a second Ctrl+C below.
        drone_controller.get_logger().info(f'User hit Ctrl+C. Attempting to land drone {drone_controller.drone_name}.')
        drone_controller.should_land = True
        try:
            # Land height is the descent distance to travel: negative current
            # altitude so the drone is commanded down to the floor. NOTE: pos[Z]
            # comes from the cf01 TF and can go NaN/stale on tracking loss.
            land_height = -drone_controller.pos[Z_DIR]
            land_resp = drone_controller.send_land_req(group_mask=drone_controller.GROUP_MASK, height=land_height, duration=drone_controller.DURATION)
            drone_controller.get_logger().info(f'Requesting drone {drone_controller.drone_name} to land (height={land_height}).')
            if land_resp is not None: time.sleep(drone_controller.DURATION.sec)
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