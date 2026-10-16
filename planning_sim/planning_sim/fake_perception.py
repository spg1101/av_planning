"""Perception stand-in: republishes other agents' ground-truth odometry as detections.

In:  odometry topics listed in `object_topics` (e.g. /opp_racecar/odom in two-agent gym)
Out: /perception/detections (vision_msgs/Detection3DArray, map frame) with optional noise
"""

import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray


class FakePerception(Node):
    def __init__(self):
        super().__init__("fake_perception")
        self.declare_parameter("object_topics", ["/opp_racecar/odom"])
        self.declare_parameter("position_noise_std", 0.05)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("object_size", [0.5, 0.3, 0.2])
        self.latest = {}
        for topic in self.get_parameter("object_topics").value:
            self.create_subscription(Odometry, topic, lambda m, t=topic: self.latest.__setitem__(t, m), 10)
        self.pub = self.create_publisher(Detection3DArray, "/perception/detections", 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)

    def tick(self):
        out = Detection3DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        noise = self.get_parameter("position_noise_std").value
        size = self.get_parameter("object_size").value
        for odom in self.latest.values():
            det = Detection3D()
            det.header = out.header
            det.bbox.center = odom.pose.pose
            det.bbox.center.position.x += random.gauss(0.0, noise)
            det.bbox.center.position.y += random.gauss(0.0, noise)
            det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = size
            out.detections.append(det)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FakePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
