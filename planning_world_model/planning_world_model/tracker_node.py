"""Object tracker (Member A, task P2.1).

In:  /perception/detections (vision_msgs/Detection3DArray), assumed in the map frame
Out: /tracking/objects (av_msgs/TrackedObjectArray)

Version 0: greedy nearest-neighbour association + finite-difference velocity.
TODO(member A): Kalman filter per track, Hungarian matching (scipy.optimize.linear_sum_assignment),
track birth/death with hit counts, and a TF transform when detections arrive in a sensor frame.
"""

import math

import rclpy
from av_msgs.msg import TrackedObject, TrackedObjectArray
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

from planning_nodes.conversions import stamp_to_seconds


class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker")
        self.declare_parameter("match_distance", 1.0)
        self.declare_parameter("max_age", 0.5)
        self.tracks = {}  # id -> dict(x, y, vx, vy, t, size)
        self.next_id = 1
        self.create_subscription(Detection3DArray, "/perception/detections", self.on_detections, 10)
        self.pub = self.create_publisher(TrackedObjectArray, "/tracking/objects", 10)

    def on_detections(self, msg: Detection3DArray):
        t = stamp_to_seconds(msg.header.stamp)
        gate = self.get_parameter("match_distance").value
        unmatched = set(self.tracks)
        for det in msg.detections:
            c = det.bbox.center.position
            best = min(
                unmatched,
                default=None,
                key=lambda i: math.hypot(self.tracks[i]["x"] - c.x, self.tracks[i]["y"] - c.y),
            )
            if (
                best is not None
                and math.hypot(self.tracks[best]["x"] - c.x, self.tracks[best]["y"] - c.y) < gate
            ):
                tr = self.tracks[best]
                dt = max(t - tr["t"], 1e-3)
                tr.update(vx=(c.x - tr["x"]) / dt, vy=(c.y - tr["y"]) / dt, x=c.x, y=c.y, t=t)
                unmatched.discard(best)
            else:
                self.tracks[self.next_id] = dict(
                    x=c.x,
                    y=c.y,
                    vx=0.0,
                    vy=0.0,
                    t=t,
                    size=(det.bbox.size.x, det.bbox.size.y, det.bbox.size.z),
                )
                self.next_id += 1
        max_age = self.get_parameter("max_age").value
        self.tracks = {i: tr for i, tr in self.tracks.items() if t - tr["t"] <= max_age}

        out = TrackedObjectArray()
        out.header = msg.header
        out.header.frame_id = "map"
        for i, tr in self.tracks.items():
            o = TrackedObject()
            o.id = i
            o.confidence = 1.0
            o.pose.position.x, o.pose.position.y = tr["x"], tr["y"]
            o.pose.orientation.w = 1.0
            o.velocity.x, o.velocity.y = tr["vx"], tr["vy"]
            o.size.x, o.size.y, o.size.z = tr["size"]
            out.objects.append(o)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
