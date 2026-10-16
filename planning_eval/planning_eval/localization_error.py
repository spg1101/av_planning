"""Live localization error vs ground truth (Member A, task P1.2).

Works the same on gym, kinematic sim, bag replay or CARLA:
  ros2 run planning_eval localization_error --ros-args -p use_sim_time:=true
Estimate = TF-equivalent pose from `estimate_topic` (default /world/ego_state, map frame).
Prints RMSE every 5 s and a summary on shutdown.
"""

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from planning_eval.metrics import match_by_time, position_rmse


def to_row(msg):
    return (
        msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
    )


class LocalizationError(Node):
    def __init__(self):
        super().__init__("localization_error")
        self.declare_parameter("estimate_topic", "/world/ego_state")
        self.declare_parameter("ground_truth_topic", "/ground_truth/odom")
        self.est, self.gt = [], []
        self.create_subscription(
            Odometry, self.get_parameter("estimate_topic").value, lambda m: self.est.append(to_row(m)), 50
        )
        self.create_subscription(
            Odometry, self.get_parameter("ground_truth_topic").value, lambda m: self.gt.append(to_row(m)), 50
        )
        self.create_timer(5.0, self.report)

    def compute(self):
        if len(self.est) < 2 or len(self.gt) < 2:
            return None
        est, gt = np.array(self.est), np.array(sorted(self.gt))
        idx = match_by_time(est[:, 0], gt[:, 0])
        ok = idx >= 0
        if not np.any(ok):
            return None
        return position_rmse(est[ok, 1:3], gt[idx[ok], 1:3]), int(ok.sum())

    def report(self):
        result = self.compute()
        if result:
            self.get_logger().info(f"position RMSE {result[0]:.3f} m over {result[1]} samples")


def main():
    rclpy.init()
    node = LocalizationError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        result = node.compute()
        if result:
            print(f"FINAL position RMSE {result[0]:.3f} m over {result[1]} samples")
        node.destroy_node()
        rclpy.try_shutdown()
