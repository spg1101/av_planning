"""Bag replay helper: send the first ground-truth pose to the EKF (/set_pose) and SLAM
(/initialpose), then exit. Without it, replayed localization starts at (0, 0) while the ground
truth starts at the car's real CARLA position, and every error number would be offset.
(Live in CARLA, the adapter does this itself before the first tick.)
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class InitialPose(Node):
    def __init__(self):
        super().__init__("initial_pose_from_ground_truth")
        self.sent = 0
        self.set_pose = self.create_publisher(PoseWithCovarianceStamped, "/set_pose", 10)
        self.initialpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 10)

    def on_gt(self, msg: Odometry):
        if self.sent >= 3:
            return
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = "map"
        out.pose.pose = msg.pose.pose
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.01
        cov[35] = 0.001
        out.pose.covariance = cov
        self.set_pose.publish(out)
        self.initialpose.publish(out)
        self.sent += 1


def main():
    rclpy.init()
    node = InitialPose()
    try:
        while rclpy.ok() and node.sent < 3:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
