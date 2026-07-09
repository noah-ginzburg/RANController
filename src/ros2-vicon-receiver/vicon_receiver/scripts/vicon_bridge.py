#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vicon_interfaces.msg import Position
from vicon_interfaces.msg import PositionList
import tf2_ros
from geometry_msgs.msg import TransformStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray, NamedPose

class ViconBridge(Node):
    def __init__(self):
        super().__init__('vicon_bridge')
        self.declare_parameter('all_drones', [''])
        self.all_drones = self.get_parameter('all_drones').value

        topics = self.get_topic_names_and_types()
        existing = {topic for topic, _ in topics}
        self.subscribers = {}

        self.get_logger().info('test')


        for name in self.all_drones:
            self.get_logger().info(f'searching {name}')
            matches = [t for t in existing if t.startswith(f'/vicon/{name}/')]
            if matches:
                self.subscribers[name] = self.create_subscription(Position, f'/vicon/{name}/{name}_segment', vicon_callback, 10)
                self.get_logger().info(f'topic of {name} found.')


        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.poses_pub = self.create_publisher(NamedPoseArray, '/poses', 10)


    def vicon_callback(self, msg: Position):
        self.get_logger().info(f'{msg.subject_name} data received')
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'mocap'
        transform.child_frame_id = msg.subject_name
        transform.transform.translation.x = msg.x_trans / 1000.0
        transform.transform.translation.y = msg.y_trans / 1000.0
        transform.transform.translation.z = msg.z_trans / 1000.0

        transform.transform.rotation.x = msg.x_rot
        transform.transform.rotation.y = msg.y_rot
        transform.transform.rotation.z = msg.z_rot
        transform.transform.rotation.w = msg.w
        self.tf_broadcaster.sendTransform(transform)

        named_pose = NamedPose()
        named_pose.name = msg.subject_name
        named_pose.pose.position.x = msg.x_trans / 1000.0
        named_pose.pose.position.y = msg.y_trans / 1000.0
        named_pose.pose.position.z = msg.z_trans / 1000.0
        named_pose.pose.orientation.x = msg.x_rot
        named_pose.pose.orientation.y = msg.y_rot
        named_pose.pose.orientation.z = msg.z_rot
        named_pose.pose.orientation.w = msg.w
        named_poses.append(named_pose)

        self.poses_pub.publish(NamedPoseArray(poses=named_poses))

def main(args=None):
    rclpy.init(args=args)
    vicon_bridge = ViconBridge()
    rclpy.spin(vicon_bridge)
    vicon_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
