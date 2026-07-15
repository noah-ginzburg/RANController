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

MARKER_HEIGHT_OFFSET = 0.0

class ViconBridge(Node):
    def __init__(self):
        super().__init__('vicon_bridge')
        self.declare_parameter('all_drones', [''])
        self.all_drones = self.get_parameter('all_drones').value

        topics = self.get_topic_names_and_types()
        existing = {topic for topic, _ in topics}
        self.subscribers = {}
        # Last raw pose per subject, used to drop frozen frames. When Tracker
        # loses the object it re-serves the last pose bit-identically (the
        # receiver never checks the SDK's Occluded flag); real tracking always
        # has sub-mm noise between frames, so exact repeats mean stale data.
        self.last_raw = {}


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
            self.get_logger().warn("nan observed in quaternion")
            return
        if abs(math.sqrt(sum(v * v for v in quat)) - 1.0) > 0.1:  # not a unit quaternion
            self.get_logger().warn("not a unit quaternion")
            return
        if msg.x_trans == 0.0 and msg.y_trans == 0.0 and msg.z_trans == 0.0:
            self.get_logger().warn("invalid 0,0,0 frame")
            return

        raw = (msg.x_trans, msg.y_trans, msg.z_trans,
               msg.x_rot, msg.y_rot, msg.z_rot, msg.w)
        if self.last_raw.get(msg.subject_name) == raw:
            self.get_logger().warn(
                f'{msg.subject_name}: frozen Vicon frame (tracking lost?), dropping',
                throttle_duration_sec=1.0)
            return
        self.last_raw[msg.subject_name] = raw

        x_m = msg.x_trans / 1000.0
        y_m = msg.y_trans / 1000.0
        z_m = (msg.z_trans - MARKER_HEIGHT_OFFSET) / 1000.0

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'mocap'
        transform.child_frame_id = msg.subject_name
        transform.transform.translation.x = x_m
        transform.transform.translation.y = y_m
        transform.transform.translation.z = z_m

        transform.transform.rotation.x = msg.x_rot
        transform.transform.rotation.y = msg.y_rot
        transform.transform.rotation.z = msg.z_rot
        transform.transform.rotation.w = msg.w
        self.tf_broadcaster.sendTransform(transform)

        named_pose = NamedPose()
        named_pose.name = msg.subject_name
        named_pose.pose.position.x = x_m
        named_pose.pose.position.y = y_m
        named_pose.pose.position.z = z_m
        named_pose.pose.orientation.x = msg.x_rot
        named_pose.pose.orientation.y = msg.y_rot
        named_pose.pose.orientation.z = msg.z_rot
        named_pose.pose.orientation.w = msg.w

        self.poses_pub.publish(NamedPoseArray(poses=[named_pose]))

def main(args=None):
    rclpy.init(args=args)
    vicon_bridge = ViconBridge()
    rclpy.spin(vicon_bridge)
    vicon_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
