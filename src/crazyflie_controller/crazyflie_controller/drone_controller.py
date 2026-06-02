import sys
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from crazyflie_interfaces.msg import FullState
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.duration import Duration as RCLDuration
from builtin_interfaces.msg import Duration
from tf2_ros import TransformListener, Buffer

#Simple indexing
X_DIR = ROLL = 0
Y_XIR = PITCH = 1
Z_DIR = YAW = 2

class DroneController(Node):
    UPDATE_RATE = 50.0  #hz
    GROUP_MASK = 0  #0 = all drones
    HEIGHT = 1.0    #Desired launch height
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

        self.create_timer(1.0 / self.UPDATE_RATE, self.update)
        self.prev_time = self.get_clock().now()
        self.now = 0.0

        self.w_abs_desired = 0.0
        self.max_speed = 0.25

        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.acc = np.array([0.0, 0.0, 0.0])

        self.vel_desired = np.array([0.0, 0.0, 0.0])

        self.movement_msg = FullState()
        self.movement_msg.acc.x = 0.0
        self.movement_msg.acc.y = 0.0
        self.movement_msg.acc.z = 0.0
        self.movement_pub = self.create_publisher(FullState, f'{self.drone_name}/cmd_full_state', 10)

        self.should_hover = True
        self.tf_buffer = tf2_ros.Buffer()
        self.prev_trans = None


    def send_takeoff_req(self, group_mask, height, duration):
        req = Takeoff.Request()
        req.group_mask = group_mask
        req.height = height
        req.duration = duration

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def update(self):
        self.now = self.get_clock().now()
        dt = (now - self.prev_time) * 1e-9

        #TESTING
        vel_desired = np.array([0.0, 0.0, 0.0])

        if (should_hover):
            self.hover()
        else:
            self.hover()
            # self.set_speeds(vel_desired)

        if use_sim_odom: _update_sim(dt)
        else: _update_real(dt)

        self.movement_msg.header.stamp = self.get_clock().now().to_msg()
        self.movement_msg.header.frame_id = self.drone_name
        self.movement_pub.publish()

        self.prev_time = self.now
        self.prev_pos = self.pos    

    def set_speeds(self, speeds):
        self.movement_msg.twist.linear.x = speeds[X_DIR]
        self.movement_msg.twist.linear.y = speeds[Y_DIR]
        self.movement_msg.twist.linear.z = speeds[Z_DIR]

    #Macro, because python has no preprocessor :(
    def hover(self): self.set_speeds(np.zeros(3))

    def _update_sim(self, dt):
        try:
            trans = self.tf_buffer.lookup_transform('mocap', f'{self.drone_name}/takeoff', rclpy.time.Time(), RCLDuration(0.5))
        except (tf2_ros.TransformException) as e:
            self.get_logger().warn(e + ", drone is being forced to hover")
            self.hover()
            return
        
        self.pos[X_DIR] = trans.translation.x
        self.pos[Y_DIR] = trans.translation.y
        self.pos[Z_DIR] = trans.translation.z

        self.vel = (self.pos - self.prev_pos)/dt

        

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


    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()