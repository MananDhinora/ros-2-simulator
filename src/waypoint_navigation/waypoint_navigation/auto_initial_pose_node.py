import math
import time

import rclpy
import tf_transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__("auto_initial_pose_node")

        # Parameters
        self.declare_parameter("default_x", 0.0)
        self.declare_parameter("default_y", 0.0)
        self.declare_parameter("default_yaw", 0.0)

        self.default_x = self.get_parameter("default_x").value
        self.default_y = self.get_parameter("default_y").value
        self.default_yaw = self.get_parameter("default_yaw").value

        # Publisher
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # Publish once after a short delay to ensure subscribers are ready
        self.timer = self.create_timer(0.5, self.publish_static_pose)

    def publish_static_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        q = tf_transformations.quaternion_from_euler(0, 0, self.default_yaw)
        pose_msg.pose.pose.position.x = self.default_x
        pose_msg.pose.pose.position.y = self.default_y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = math.radians(10) ** 2

        self.initialpose_pub.publish(pose_msg)
        self.get_logger().info(
            f"[AUTO INIT] Static initial pose sent: x={self.default_x:.2f}, y={self.default_y:.2f}, yaw={math.degrees(self.default_yaw):.1f}°"
        )

        self.timer.cancel()  # only publish once


def main(args=None):
    rclpy.init(args=args)
    node = AutoInitialPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
