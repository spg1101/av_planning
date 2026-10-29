"""Behavior planner node (Lead, task P2.5).

In:  /world/ego_state (Odometry), /world/obstacles (TrackedObjectArray), /planning/route (Path)
Out: /planning/behavior (av_msgs/BehaviorCommand) at ~10 Hz

- FOLLOW_LANE with an adaptive-cruise speed limit when a vehicle is ahead in our lane
- STOP with a stop point `stop_margin` metres before a stopped obstacle
TODO(lead): measure "ahead" along the route (Frenet s) instead of the ego heading, YIELD at
junctions, traffic lights (Phase 3).
"""

import math

import rclpy
from av_msgs.msg import BehaviorCommand, TrackedObjectArray
from nav_msgs.msg import Odometry
from rclpy.node import Node

from planning_core.behavior.state_machine import (
    BehaviorInputs,
    BehaviorParams,
    BehaviorState,
    BehaviorStateMachine,
    follow_speed_limit,
)
from planning_nodes.conversions import quaternion_to_yaw


class BehaviorNode(Node):
    def __init__(self):
        super().__init__("behavior_planner")
        d = self.declare_parameter
        d("rate_hz", 10.0)
        d("speed_limit", 3.0)  # m/s
        d("lane_half_width", 0.4)  # m: obstacles closer than this to our heading line are "in lane"
        d("stop_distance", 2.0)
        d("clear_distance", 4.0)
        d("stop_margin", 1.0)  # m: stop this far before the obstacle
        d("follow_range", 15.0)
        d("min_gap", 1.0)
        d("time_gap", 1.0)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.params = BehaviorParams(
            stop_distance=gp("stop_distance"),
            clear_distance=gp("clear_distance"),
            follow_range=gp("follow_range"),
            min_gap=gp("min_gap"),
            time_gap=gp("time_gap"),
        )
        self.sm = BehaviorStateMachine(self.params)
        self.ego = None
        self.obstacles = None

        self.create_subscription(Odometry, "/world/ego_state", lambda m: setattr(self, "ego", m), 10)
        self.create_subscription(
            TrackedObjectArray, "/world/obstacles", lambda m: setattr(self, "obstacles", m), 10
        )
        self.pub = self.create_publisher(BehaviorCommand, "/planning/behavior", 10)
        self.create_timer(1.0 / gp("rate_hz"), self.tick)

    def nearest_obstacle_ahead(self):
        """Gap to and speed of the closest obstacle in our lane ahead (gap = centre distance
        minus half the obstacle length)."""
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
            gap = ahead - 0.5 * o.size.x
            if ahead > 0.0 and abs(lateral) < half_w + 0.5 * o.size.y and gap < best:
                # speed along our heading (a crossing car is not a lead vehicle)
                best = gap
                speed = math.cos(yaw) * o.velocity.x + math.sin(yaw) * o.velocity.y
        return best, speed

    def tick(self):
        if self.ego is None:
            return
        gap, lead_speed = self.nearest_obstacle_ahead()
        state = self.sm.step(BehaviorInputs(obstacle_ahead_dist=gap, obstacle_ahead_speed=lead_speed))
        limit = float(self.get_parameter("speed_limit").value)

        msg = BehaviorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.state = int(state)
        msg.speed_limit = follow_speed_limit(gap, lead_speed, limit, self.params)
        if state == BehaviorState.STOP and math.isfinite(gap):
            p = self.ego.pose.pose
            yaw = quaternion_to_yaw(p.orientation)
            along = max(0.0, gap - float(self.get_parameter("stop_margin").value))
            msg.has_stop_point = True
            msg.stop_point.x = p.position.x + along * math.cos(yaw)
            msg.stop_point.y = p.position.y + along * math.sin(yaw)
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
