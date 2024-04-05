#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path

class PoseTransformNode(Node):

    def __init__(self):
        super().__init__('pose_transformer')
        self.subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        # Publisher for the NED transformed pose in PoseWithCovarianceStamped format
        self.publisher_pose_ned = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose_ned', 10)
        
        # Publishers for the paths in both ENU and NED
        self.publisher_path_enu = self.create_publisher(Path, '/path_enu', 10)
        self.publisher_path_ned = self.create_publisher(Path, '/path_ned', 10)
        self.enu_path = Path()
        self.ned_path = Path()

    def pose_callback(self, msg):
        # Create a PoseStamped message for the original ENU path
        enu_pose_stamped = PoseStamped()
        enu_pose_stamped.header = msg.header
        enu_pose_stamped.pose = msg.pose.pose
        enu_pose_stamped.header.stamp = self.get_clock().now().to_msg()        
        self.enu_path.header.frame_id = "map"
        self.enu_path.header.stamp = self.get_clock().now().to_msg()
        self.enu_path.poses.append(enu_pose_stamped)
        self.publisher_path_enu.publish(self.enu_path)

        # Transform the pose from ENU to NED
        ned_pose = PoseWithCovarianceStamped()
        ned_pose.header = msg.header
        ned_pose.header.frame_id = "map"  # Set to map frame
        ned_pose.pose.pose.position.x = msg.pose.pose.position.y
        ned_pose.pose.pose.position.y = msg.pose.pose.position.x
        ned_pose.pose.pose.position.z = -msg.pose.pose.position.z
        ned_pose.pose.pose.orientation.x = msg.pose.pose.orientation.y
        ned_pose.pose.pose.orientation.y = msg.pose.pose.orientation.x
        ned_pose.pose.pose.orientation.z = -msg.pose.pose.orientation.z
        ned_pose.pose.pose.orientation.w = msg.pose.pose.orientation.w
        ned_pose.pose.covariance = msg.pose.covariance

        # Publish the NED pose in PoseWithCovarianceStamped format
        self.publisher_pose_ned.publish(ned_pose)

        # Create a PoseStamped message for the NED path
        ned_pose_stamped = PoseStamped()
        ned_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ned_pose_stamped.header = msg.header
        ned_pose_stamped.header.frame_id = "map"  # Set to map frame
        ned_pose_stamped.pose = ned_pose.pose.pose
        self.ned_path.header.stamp = self.get_clock().now().to_msg()
        self.ned_path.header.frame_id = "map"
        self.ned_path.poses.append(ned_pose_stamped)
        self.publisher_path_ned.publish(self.ned_path)

def main(args=None):
    rclpy.init(args=args)

    pose_transformer = PoseTransformNode()
    rclpy.spin(pose_transformer)
    pose_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
