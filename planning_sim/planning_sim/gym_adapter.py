"""Platform adapter for f1tenth_gym_ros (stand-in until the integration team's adapter).

f1tenth_gym_ros publishes /scan (frame ego_racecar/laser), /ego_racecar/odom (ground truth)
and its own TF tree map -> ego_racecar/base_link. Our stack uses map -> odom -> base_link,
so this node translates:

  /scan              -> /sensors/scan   (frame_id rewritten to `laser`)
  /ego_racecar/odom  -> /vehicle/odom   (twist only, optional noise: what wheel odometry gives)
                     -> /ground_truth/odom (for evaluation only, never used by the stack)
  static TF base_link -> laser

The gym's own TF frames (ego_racecar/*) stay separate, so the two trees never conflict.
"""

import random

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


class GymAdapter(Node):
    def __init__(self):
        super().__init__("gym_adapter")
        self.declare_parameter("laser_x_offset", 0.275)  # check scan_distance_to_base_link in sim.yaml
        self.declare_parameter("speed_noise_std", 0.02)  # m/s
        self.declare_parameter("yaw_rate_noise_std", 0.01)  # rad/s

        self.scan_pub = self.create_publisher(LaserScan, "/sensors/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/vehicle/odom", 10)
        self.gt_pub = self.create_publisher(Odometry, "/ground_truth/odom", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/ego_racecar/odom", self.on_odom, 10)

        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser"
        t.transform.translation.x = float(self.get_parameter("laser_x_offset").value)
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)

    def on_scan(self, msg: LaserScan):
        msg.header.frame_id = "laser"
        self.scan_pub.publish(msg)

    def on_odom(self, msg: Odometry):
        gt = Odometry()
        gt.header = msg.header
        gt.header.frame_id = "map"
        gt.child_frame_id = "base_link"
        gt.pose = msg.pose
        gt.twist = msg.twist
        self.gt_pub.publish(gt)

        wheel = Odometry()
        wheel.header.stamp = msg.header.stamp
        wheel.header.frame_id = "odom"
        wheel.child_frame_id = "base_link"
        wheel.twist.twist.linear.x = msg.twist.twist.linear.x + random.gauss(
            0.0, self.get_parameter("speed_noise_std").value
        )
        wheel.twist.twist.angular.z = msg.twist.twist.angular.z + random.gauss(
            0.0, self.get_parameter("yaw_rate_noise_std").value
        )
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = 0.01, 0.01, 0.01
        wheel.twist.covariance = cov
        self.odom_pub.publish(wheel)


def main():
    rclpy.init()
    node = GymAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
