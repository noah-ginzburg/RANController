import rclpy
from rclpy.node import Node
from crazyflie_py import Crazyswarm


def main():
    # Initialize Crazyswarm (this sets up ROS2 internally)
    swarm = Crazyswarm()
    node = rclpy.create_node("takeoff_hover_land")

    # Declare and read parameters
    node.declare_parameter('height', 1.0)
    node.declare_parameter('goto_x', 0.5)
    node.declare_parameter('goto_y', 0.5)

    height = node.get_parameter('height').value
    goto_x = node.get_parameter('goto_x').value
    goto_y = node.get_parameter('goto_y').value

    # Flight logic

    

    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    TAKEOFF_DURATION = 2.5
    GOTO_DURATION = 3.0

    for i in range(4):
        input("press enter")

        cf.takeoff(targetHeight=height, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION)

        goto_x = node.get_parameter('goto_x').value
        goto_y = node.get_parameter('goto_y').value

        pos = [[goto_x, goto_y, height], [0., 0., height]]
        cf.goTo(pos[i%2], 0., duration=GOTO_DURATION)
        timeHelper.sleep(GOTO_DURATION)

        cf.land(targetHeight=0.04, duration=2.5)
        timeHelper.sleep(TAKEOFF_DURATION)

    node.destroy_node()


if __name__ == '__main__':
    main()