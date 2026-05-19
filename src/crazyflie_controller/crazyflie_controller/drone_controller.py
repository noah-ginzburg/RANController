import sys
#from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
import numpy as np


class DroneController(Node):
    UPDATE_RATE = 50.0

    def __init__(self):
        super().__init__('drone_controller')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

        self.create_timer(1.0 / self.UPDATE_RATE, self.update)

        self.w = 0.0  #fill in with compass later
        self.w_abs_desired = 0.0
        self.max_speed = 0.25

        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.acc = np.array([0.0, 0.0, 0.0])

        self.vel_desired = np.array([0.0, 0.0, 0.0])


    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def update(self):
        # get current state
        # compute desired velocity
        # send command to drone
        pass


def main(args=None):
    rclpy.init(args=args)

    drone_controller = DroneController()
    response = drone_controller.send_request(int(sys.argv[1]), int(sys.argv[2]))
    drone_controller.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()