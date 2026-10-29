"""Perception stand-in: CARLA ground-truth objects -> noisy detections, like a real detector.

In:  /ground_truth/objects (av_msgs/TrackedObjectArray, map frame, from the CARLA adapter)
     /ground_truth/odom (for the detection range)
Out: /perception/detections (vision_msgs/Detection3DArray, map frame)

Knobs make it imperfect on purpose so the tracker and planner are tested against realistic
input: position noise, a limited range, and randomly missed detections. Swap this node for
the perception team's model by not launching it (perception:=model in carla.launch.py).
"""

import math

import numpy as np
import rclpy
from av_msgs.msg import TrackedObjectArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

CLASS_NAMES = {0: "unknown", 1: "car", 2: "pedestrian", 3: "cyclist"}


class FakePerception(Node):
    def __init__(self):
        super().__init__("fake_perception")
        self.declare_parameter("position_noise_std", 0.2)  # m
        self.declare_parameter("max_range", 50.0)  # m
        self.declare_parameter("miss_probability", 0.05)
        self.declare_parameter("seed", 0)
        self.rng = np.random.default_rng(self.get_parameter("seed").value)
        self.ego = None
        self.create_subscription(Odometry, "/ground_truth/odom", lambda m: setattr(self, "ego", m), 10)
        self.create_subscription(TrackedObjectArray, "/ground_truth/objects", self.on_objects, 10)
        self.pub = self.create_publisher(Detection3DArray, "/perception/detections", 10)

    def on_objects(self, msg: TrackedObjectArray):
        if self.ego is None:
            return
        ex, ey = self.ego.pose.pose.position.x, self.ego.pose.pose.position.y
        noise = self.get_parameter("position_noise_std").value
        max_range = self.get_parameter("max_range").value
        miss = self.get_parameter("miss_probability").value
        out = Detection3DArray()
        out.header = msg.header
        for o in msg.objects:
            if math.hypot(o.pose.position.x - ex, o.pose.position.y - ey) > max_range:
                continue
            if self.rng.random() < miss:
                continue
            det = Detection3D()
            det.header = msg.header
            det.bbox.center = o.pose
            det.bbox.center.position.x += self.rng.normal(0.0, noise)
            det.bbox.center.position.y += self.rng.normal(0.0, noise)
            det.bbox.size = o.size
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = CLASS_NAMES.get(o.classification, "unknown")
            hyp.hypothesis.score = 1.0
            det.results.append(hyp)
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
