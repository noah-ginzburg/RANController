import sys
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from crazyflie_interfaces.msg import FullState
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.duration import Duration as RCLDuration
from builtin_interfaces.msg import Duration
from geometry_msgs import Twist

#Simple indexing
X_DIR = ROLL = 0
Y_XIR = PITCH = 1
Z_DIR = YAW = 2

class DroneController(Node):
    UPDATE_RATE = 50.0
    GROUP_MASK = 0
    HEIGHT = 1.0    #Desired launch height
    DURATION = Duration(sec=3, nanosec=0)   #Time to reach the desired height
    DEADZONE_VELOCITY = 0.01    #any velocity slow than this will make the drone hover in place

    KP = 1
    KI = 1
    KD = 1
     
    def __init__(self):
        super().__init__('drone_controller')
        self.declare_parameter('drone_name', 'cf01') # 2. declare
        self.drone_name = self.get_parameter('drone_name').value    #drone name parameter

        self.cli = self.create_client(Takeoff, f'{self.drone_name}/takeoff')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.create_timer(1.0 / self.UPDATE_RATE, self.update)
        self.prev_time = self.get_clock().now()

        self.w_abs_desired = 0.0
        self.max_speed = 0.25

        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.acc = np.array([0.0, 0.0, 0.0])

        self.vel_desired = np.array([0.0, 0.0, 0.0])

        # self.movement_msg = FullState()
        # self.movement_msg.acc.x = 0.0
        # self.movement_msg.acc.y = 0.0
        # self.movement_msg.acc.z = 0.0
        # self.movement_pub = self.create_publisher(FullState, f'{self.drone_name}/cmd_full_state', 10)

        self.movement_msg = Twist()
        self.movement_pub = self.create_publisher(FullState, f'{self.drone_name}/cmd_vel_legacy', 10)


    def send_takeoff_req(self, group_mask, height, duration):
        req = Takeoff.Request()
        req.group_mask = group_mask
        req.height = height
        req.duration = duration

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def update(self):
        self.movement_msg.header.stamp = self.get_clock().now().to_msg()
        self.movement_msg.header.frame_id = self.drone_name

        now = self.get_clock().now()
        dt = (now - self.prev_time) * 1e-9

        if (np.linalg.norm(self.vel) < self.DEADZONE_VELOCITY):
            self.set_hover_speeds()
        else:
            # self.set_movement_speeds()
            self.set_hover_speeds()

        self.movement_pub.publish()
        
        self.prev_time = now

        

    def set_hover_speeds():
        self.vel_desired = np.array([0.0, 0.0, 0.0])
        self.set_msg_speeds()


    def set_movement_speeds():
        #implement RAN
        self.vel_desired = np.array([0.0, 0.0, self.max_speed])
        self.set_msg_speeds()


    def set_msg_speeds():
        self.movement_msg.twist.x = vel_desired[X_DIR]
        self.movement_msg.twist.y = vel_desired[Y_DIR]
        self.movement_msg.twist.y = vel_desired[Z_DIR]


def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    response = drone_controller.send_takeoff_req(group_mask=drone_controller.GROUP_MASK, height=drone_controller.HEIGHT, duration=drone_controller.DURATION)
    if response is not None:
        drone_controller.get_logger().info('Takeoff command executed successfully')
    else:
        drone_controller.get_logger().error('Takeoff service call failed')


    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()