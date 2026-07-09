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

MARKER_HEIGHT_OFFSET = 35.083595275878906

class ViconBridge(Node):
    def __init__(self):
        super().__init__('vicon_bridge')
        self.declare_parameter('all_drones', [''])
        self.all_drones = self.get_parameter('all_drones').value

        topics = self.get_topic_names_and_types()
        existing = {topic for topic, _ in topics}
        self.subscribers = {}


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


    def vicon_callback(self, msg: Position):
        # self.get_logger().info(f'{msg.subject_name} data received')

        # Drop occluded / invalid frames. When Vicon loses the rigid body it
        # reports translation (0,0,0) and a NaN/zero quaternion. Forwarding that
        # teleports the drone's estimator to the origin and it crashes. A brief
        # gap is fine -- the onboard Kalman filter coasts on gyro/accel.
        quat = (msg.x_rot, msg.y_rot, msg.z_rot, msg.w)
        if any(math.isnan(v) for v in quat):
            return
        if abs(math.sqrt(sum(v * v for v in quat)) - 1.0) > 0.1:  # not a unit quaternion
            return
        if msg.x_trans == 0.0 and msg.y_trans == 0.0 and msg.z_trans == 0.0:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'mocap'
        transform.child_frame_id = msg.subject_name
        transform.transform.translation.x = msg.x_trans / 1000.0
        transform.transform.translation.y = msg.y_trans / 1000.0
        transform.transform.translation.z = (msg.z_trans - MARKER_HEIGHT_OFFSET) / 1000.0

        transform.transform.rotation.x = msg.x_rot
        transform.transform.rotation.y = msg.y_rot
        transform.transform.rotation.z = msg.z_rot
        transform.transform.rotation.w = msg.w
        self.tf_broadcaster.sendTransform(transform)

        named_pose = NamedPose()
        named_pose.name = msg.subject_name
        named_pose.pose.position.x = msg.x_trans / 1000.0
        named_pose.pose.position.y = msg.y_trans / 1000.0
        named_pose.pose.position.z = (msg.z_trans - MARKER_HEIGHT_OFFSET) / 1000.0
        # NaN orientation -> crazyflie_server sends position-only (extPos) instead
        # of full pose (extPose), so a misaligned Vicon rigid-body yaw can't corrupt
        # the firmware heading estimate. Firmware estimates yaw from the gyro.
        named_pose.pose.orientation.x = float('nan')
        named_pose.pose.orientation.y = float('nan')
        named_pose.pose.orientation.z = float('nan')
        named_pose.pose.orientation.w = float('nan')

        self.poses_pub.publish(NamedPoseArray(poses=[named_pose]))

def main(args=None):
    rclpy.init(args=args)
    vicon_bridge = ViconBridge()
    rclpy.spin(vicon_bridge)
    vicon_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
