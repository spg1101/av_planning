"""Perfect localization from simulator ground truth (localization:=ground_truth).

Lets planning work in CARLA before (or while) the EKF and SLAM are being tuned, and isolates
planning bugs from localization bugs. Publishes exactly what the real localization does:

  TF odom -> base_link       (with map -> odom = identity, from a static publisher in the launch)
  /odometry/filtered         (nav_msgs/Odometry, odom frame)

In:  /ground_truth/odom (map frame, from the CARLA adapter or kinematic_sim)
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GroundTruthLocalization(Node):
    def __init__(self):
        super().__init__("ground_truth_localization")
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 10)

    def on_gt(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        p = msg.pose.pose
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self.tf.sendTransform(t)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link"
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GroundTruthLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
