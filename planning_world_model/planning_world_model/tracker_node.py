"""Object tracker (Member A, task P2.1).

In:  /perception/detections (vision_msgs/Detection3DArray), assumed in the map frame
Out: /tracking/objects (av_msgs/TrackedObjectArray)

Version 0: greedy nearest-neighbour association on predicted positions + alpha-beta filter.
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
        self.declare_parameter("match_distance", 2.0)
        self.declare_parameter("max_age", 0.5)
        self.declare_parameter("alpha", 0.5)  # position correction weight
        self.declare_parameter("beta", 0.1)  # velocity correction weight
        self.tracks = {}  # id -> dict(x, y, vx, vy, t, size)
        self.next_id = 1
        self.create_subscription(Detection3DArray, "/perception/detections", self.on_detections, 10)
        self.pub = self.create_publisher(TrackedObjectArray, "/tracking/objects", 10)

    def on_detections(self, msg: Detection3DArray):
        """Alpha-beta filter per track: predict with the track's velocity, gate on the
        predicted position, then blend the measurement in. Smooths noisy detections so the
        velocity is usable by behavior and prediction (raw differencing would not be)."""
        t = stamp_to_seconds(msg.header.stamp)
        gate = self.get_parameter("match_distance").value
        alpha, beta = self.get_parameter("alpha").value, self.get_parameter("beta").value
        predicted = {
            i: (tr["x"] + tr["vx"] * (t - tr["t"]), tr["y"] + tr["vy"] * (t - tr["t"]))
            for i, tr in self.tracks.items()
        }
        unmatched = set(self.tracks)
        for det in msg.detections:
            c = det.bbox.center.position
            best = min(
                unmatched,
                default=None,
                key=lambda i: math.hypot(predicted[i][0] - c.x, predicted[i][1] - c.y),
            )
            if best is not None and math.hypot(predicted[best][0] - c.x, predicted[best][1] - c.y) < gate:
                tr = self.tracks[best]
                dt = max(t - tr["t"], 1e-3)
                px, py = predicted[best]
                rx, ry = c.x - px, c.y - py
                tr.update(
                    x=px + alpha * rx,
                    y=py + alpha * ry,
                    vx=tr["vx"] + beta * rx / dt,
                    vy=tr["vy"] + beta * ry / dt,
                    t=t,
                )
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
