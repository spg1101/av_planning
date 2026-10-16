"""Behavior planner node (Lead, task P2.5).

In:  /world/ego_state (Odometry), /world/obstacles (TrackedObjectArray), /planning/route (Path)
Out: /planning/behavior (av_msgs/BehaviorCommand) at ~10 Hz
"""

import math

import rclpy
from av_msgs.msg import BehaviorCommand, TrackedObjectArray
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from planning_core.behavior.state_machine import BehaviorInputs, BehaviorParams, BehaviorStateMachine
from planning_nodes.conversions import quaternion_to_yaw


class BehaviorNode(Node):
    def __init__(self):
        super().__init__("behavior_planner")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("speed_limit", 3.0)
        self.declare_parameter("lane_half_width", 0.4)
        self.declare_parameter("stop_distance", 2.0)
        self.sm = BehaviorStateMachine(
            BehaviorParams(stop_distance=self.get_parameter("stop_distance").value)
        )
        self.ego = None
        self.obstacles = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "/world/ego_state", lambda m: setattr(self, "ego", m), 10)
        self.create_subscription(
            TrackedObjectArray, "/world/obstacles", lambda m: setattr(self, "obstacles", m), 10
        )
        self.create_subscription(Path, "/planning/route", lambda m: None, latched)
        self.pub = self.create_publisher(BehaviorCommand, "/planning/behavior", 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)

    def nearest_obstacle_ahead(self):
        """Distance and speed of the closest obstacle in front of us, within our lane width.

        TODO(lead): measure along the route (Frenet s) instead of in the ego frame.
        """
        if self.ego is None or self.obstacles is None:
            return float("inf"), 0.0
        p = self.ego.pose.pose
        yaw = quaternion_to_yaw(p.orientation)
        half_w = self.get_parameter("lane_half_width").value
        best, speed = float("inf"), 0.0
        for o in self.obstacles.objects:
            dx, dy = o.pose.position.x - p.position.x, o.pose.position.y - p.position.y
            ahead = math.cos(yaw) * dx + math.sin(yaw) * dy
            lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if ahead > 0.0 and abs(lateral) < half_w and ahead < best:
                best, speed = ahead, math.hypot(o.velocity.x, o.velocity.y)
        return best, speed

    def tick(self):
        dist, speed = self.nearest_obstacle_ahead()
        state = self.sm.step(BehaviorInputs(obstacle_ahead_dist=dist, obstacle_ahead_speed=speed))
        msg = BehaviorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.state = int(state)
        msg.speed_limit = float(self.get_parameter("speed_limit").value)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
