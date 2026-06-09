import sys
import time
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from crazyflie_interfaces.msg import FullState
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.duration import Duration as RCLDuration
from builtin_interfaces.msg import Duration
import tf2_ros
from geometry_msgs.msg import Twist, Vector3

#Simple indexing
X_DIR = ROLL = 0
Y_DIR = PITCH = 1
Z_DIR = YAW = 2

#Constant hovering speed    
HOVER_SPEED_SIM = 0.23
# HOVER_SPEED_SIM = 0.0


class DroneController(Node):
    UPDATE_RATE = 50.0  #hz
    GROUP_MASK = 0  #0 = all drones
    HEIGHT = 1.0   #Desired launch height
    DURATION = Duration(sec=3, nanosec=0)   #Time to reach the desired height


    def __init__(self):
        super().__init__('drone_controller')
        self.declare_parameter('drone_name', 'cf01') # declare
        self.declare_parameter('use_sim_odom', True) # declare

        self.drone_name = self.get_parameter('drone_name').value    #drone name parameter
        self.use_sim_odom = self.get_parameter('use_sim_odom').value

        self.cli = self.create_client(Takeoff, f'{self.drone_name}/takeoff')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.des_heading_sub = self.create_subscription(Vector3, f'{self.drone_name}/desired_heading', self.des_heading_callback, 10)

        self.create_timer(1.0 / self.UPDATE_RATE, self.update)
        self.prev_time = self.get_clock().now()
        self.now = 0.0

        self.w_abs_desired = 0.0
        self.max_speed = 2.0

        self.pos = np.array([0.0, 0.0, 0.0])
        self.prev_pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.acc = np.array([0.0, 0.0, 0.0])

        self.vel_desired = np.array([0.0, 0.0, 0.0])

        self.movement_msg = FullState()
        self.movement_msg.acc.x = 0.0
        self.movement_msg.acc.y = 0.0
        self.movement_msg.acc.z = 0.0
        self.movement_pub = self.create_publisher(FullState, f'{self.drone_name}/cmd_full_state', 10)

        self.should_hover = False
        self.taking_off = True
        self.tf_ready = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.prev_trans = None

        self.last_teleop_msg_time = self.get_clock().now()
        self.teleop_timeout = RCLDuration(seconds=0.5)


    def send_takeoff_req(self, group_mask, height, duration):
        req = Takeoff.Request()
        req.group_mask = group_mask
        req.height = height
        req.duration = duration

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def cmd_vel_callback(self, msg: Twist):
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

        v = np.array([msg.x, msg.y, msg.z])
        norm = np.linalg.norm(v)
        if norm > 0:
            v = self._snap_vector(v / norm)

        self.set_speeds(self.max_speed * v)

        self.get_logger().info(f'setting speeds to {self.max_speed * v}')


    def update(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9

        

        self.vel_desired = np.array([0.0, 0.0, 0.0])

        time_from_teleop = now - self.last_teleop_msg_time

        if time_from_teleop > self.teleop_timeout or self.should_hover:
            self.hover()
        # else:
            # self.get_logger().warn("No velocity command sent from controller server. Another source might be sending velocity    commands.")

        if self.use_sim_odom: self._update_sim(dt)
        else: self._update_real(dt)

        if not self.tf_ready:
            self.get_logger().warn("No tf ready, exiting update loop")
            return

        

        self.movement_msg.header.stamp = self.get_clock().now().to_msg()
        self.movement_msg.header.frame_id = self.drone_name
        self.movement_pub.publish(self.movement_msg)

        self.prev_time = now    
        self.prev_pos = self.pos

    def set_speeds(self, lin_speeds, ang_speeds=None):
        self.movement_msg.twist.linear.x = lin_speeds[X_DIR]
        self.movement_msg.twist.linear.y = lin_speeds[Y_DIR]
        self.movement_msg.twist.linear.z = lin_speeds[Z_DIR] + HOVER_SPEED_SIM

        if not ang_speeds == None:
            self.movement_msg.twist.angular.x = ang_speeds[ROLL]
            self.movement_msg.twist.angular.y = ang_speeds[PITCH]
            self.movement_msg.twist.angular.z = ang_speeds[YAW]


    #Macro, because python has no preprocessor :(
    def hover(self): self.set_speeds(np.array([0.0, 0.0, 0.0]))

    def _update_sim(self, dt):
        if not self.tf_buffer.can_transform('mocap', self.drone_name, rclpy.time.Time()): 
            return
        try:
            trans = self.tf_buffer.lookup_transform('mocap', f'{self.drone_name}', rclpy.time.Time(), RCLDuration(seconds=1.0))
        except (tf2_ros.TransformException) as e:
            self.get_logger().warn(str(e) + ", drone is being forced to hover")
            self.should_hover = True
            return
        
        self.movement_msg.pose.position.x = self.pos[X_DIR] = trans.transform.translation.x
        self.movement_msg.pose.position.y = self.pos[Y_DIR] = trans.transform.translation.y
        self.movement_msg.pose.position.z = self.pos[Z_DIR] = trans.transform.translation.z
        self.movement_msg.pose.orientation = trans.transform.rotation

        self.vel = (self.pos - self.prev_pos)/dt
        self.tf_ready = True
        self.prev_trans = trans

    def _update_real(self):
        test = 1


def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()

    #
    response = drone_controller.send_takeoff_req(group_mask=drone_controller.GROUP_MASK, height=drone_controller.HEIGHT, duration=drone_controller.DURATION)
    if response is not None:
        drone_controller.get_logger().info('Takeoff command executed successfully')
    else:
        drone_controller.get_logger().error('Takeoff service call failed')

    time.sleep(drone_controller.DURATION.sec)

    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()