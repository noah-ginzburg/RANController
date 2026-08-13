import sys
import signal
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool

from crazyflie_interfaces.srv import Arm, GoTo, Land, Takeoff
from crazyflie_py.crazyflie import TimeHelper
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage

TAKEOFF_DURATION = 3.0
GOTO_DURATION = 5.0

def arrayToGeometryPoint(a):
    result = Point()
    result.x = a[0]
    result.y = a[1]
    result.z = a[2]
    return result

class Goto(Node):
    def __init__(self):
        super().__init__("goto_node")
        self.time_helper = TimeHelper(self)
        self.landing_pad = np.array([0.0, 0.0, 1.0])
        self.start_cf_pos = None
        self.current_pose = None
        self.nominal_vel = 0.2 # m/s
        max_inital_dist = 2
        GOTO_DURATION_WORST = 4

        self.aborted = False

        prefix = '/cf06'

        self.tf_sub    = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.odom_sub  = self.create_subscription(PoseStamped, prefix + '/pose', self.cf_pose, 10)
        self.abort_sub = self.create_subscription(Bool, '/abort_goto', self.abort, 10)
        self.start_sub = self.create_subscription(Bool, '/crazyflie/start_mission', self.start, 10)

        self.goto_time = self.create_timer(GOTO_DURATION_WORST, self.goto_callback)
        
        self.has_taken_off = False
        self.is_aborted    = False
        self.is_landing    = False
        self.start_mission  = False

        
        self.get_logger().info(f"here")
        self.emergencyService = self.create_client(Empty, prefix + '/emergency')
        while not self.emergencyService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('1 service not available, waiting again...')
        self.takeoffService = self.create_client(Takeoff, prefix + '/takeoff')
        while not self.takeoffService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('2 service not available, waiting again...')
        self.landService = self.create_client(Land, prefix + '/land')
        while not self.landService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('3 service not available, waiting again...')
        self.goToService = self.create_client(GoTo, prefix + '/go_to')
        while not self.goToService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('4 service not available, waiting again...')
        self.get_logger().info(f"here2")




    
    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "landing_pad_mocap":
                t = transform.transform.translation
                self.landing_pad[0] = t.x
                self.landing_pad[1] = t.y
                # Optional debug log
                # self.get_logger().debug(f"Landing pad updated: {self.landing_pad}")

    def start(self, msg):
        if msg.data:
            self.get_logger().info(f"starting mission!!!")
            self.start_mission = True

    def abort(self, msg):
        
        if not self.aborted:
            self.get_logger().info(f"aborting!!!")
            self.aborted = True

            if self.start_cf_pos is not None:
                self.goTo(self.start_cf_pos, 0.0, GOTO_DURATION)
                self.time_helper.sleep(GOTO_DURATION)
                self.get_logger().info(f"aborting with cf pos!!!")
            if self.start_cf_pos is None:
                self.goTo([0.0, 0.0, 1.0], 0.0, GOTO_DURATION)
                self.time_helper.sleep(GOTO_DURATION)
                self.get_logger().info(f"aborting without cf pos!!!")

            self.land(targetHeight=0.04, duration=TAKEOFF_DURATION)
            self.time_helper.sleep(TAKEOFF_DURATION)
    

    def goto_callback(self):
        
        target = np.array([self.landing_pad[0], self.landing_pad[1], 1.0])
        distance = np.linalg.norm(self.current_pose - target)


        if not self.has_taken_off and not self.aborted and self.start_mission:
            self.takeoff(1.0, TAKEOFF_DURATION)
            self.has_taken_off = True

        elif distance < 0.1 and not self.is_landing and self.has_taken_off and not self.aborted and self.start_mission:
            self.get_logger().info(f"close to landing pad, landing now")
            self.is_landing = True
            self.land(targetHeight=0.04, duration=TAKEOFF_DURATION)

        elif not self.aborted and self.has_taken_off and self.start_mission:
            self.get_logger().info(f"starting main action")
            if distance < 0.5:
                new_pose = target
            else:
                new_pose = self.current_pose + (target - self.current_pose)/2
            

            GOTO_DURATION = np.linalg.norm(new_pose - self.current_pose) / self.nominal_vel
            self.get_logger().info(f"going to landing pad at {target}")
            self.goTo(new_pose, 0.0, GOTO_DURATION)

    # def main_action(self):
    #     self.takeoff(1.0, TAKEOFF_DURATION)
    #     self.time_helper.sleep(TAKEOFF_DURATION)
    #     self.has_taken_off = True

    #     for i in range(3):
    #         if self.aborted:
    #             break
    #         target = np.array([self.landing_pad[0], self.landing_pad[1], 1.0])
    #         self.get_logger().info(f"going to landing pad at {target}")
    #         self.goTo(target, 0.0, GOTO_DURATION)
    #         self.time_helper.sleep(GOTO_DURATION)

    #     self.state = 0
    #     if not self.aborted:
    #         self.land(targetHeight=0.04, duration=TAKEOFF_DURATION)
    #         self.time_helper.sleep(TAKEOFF_DURATION)

    def landing_pad_pose(self, msg):
        self.landing_pad[0] = msg.pose.position.x
        self.landing_pad[1] = msg.pose.position.y
        #self.landing_pad[2] = msg.pose.position.z

    def cf_pose(self, msg):
        if self.start_cf_pos is None:
            self.start_cf_pos = np.array([msg.pose.position.x, msg.pose.position.y, 1.0])
            self.get_logger().info(f"added initial pos {self.start_cf_pos}")

        self.current_pose = np.array([msg.pose.position.x, msg.pose.position.y, 1.0])

    def takeoff(self, targetHeight, duration, groupMask=0):
        req = Takeoff.Request()
        req.group_mask = groupMask
        req.height = targetHeight
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.takeoffService.call_async(req)

    def land(self, targetHeight, duration, groupMask=0):
        req = Land.Request()
        req.group_mask = groupMask
        req.height = targetHeight
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.landService.call_async(req)

    def goTo(self, goal, yaw, duration, relative=False, groupMask=0):
        req = GoTo.Request()
        req.group_mask = groupMask
        req.relative = relative
        req.goal = arrayToGeometryPoint(goal)
        req.yaw = float(yaw)
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.goToService.call_async(req)

def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    agents = Goto()

    executor = MultiThreadedExecutor()
    executor.add_node(agents)

    executor.spin()

    agents.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()