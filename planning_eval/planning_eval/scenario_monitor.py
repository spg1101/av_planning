"""Scenario pass/fail judge for CARLA runs (scenario suite, task P3.6).

PASS: the car stops within goal_tolerance of the goal before the timeout, with no collision.
FAIL: any collision, or the timeout (simulation seconds) runs out.

Writes one JSON result per run to results_dir and then exits, which ends the launch
(carla.launch.py shuts everything down when this node exits), so scripts/run_scenarios.sh
can run a whole suite unattended.

In: /ground_truth/odom, /goal_pose (latched), /ground_truth/collision
"""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class ScenarioMonitor(Node):
    def __init__(self):
        super().__init__("scenario_monitor")
        self.declare_parameter("scenario_name", "unnamed")
        self.declare_parameter("timeout", 120.0)  # s of simulation time
        self.declare_parameter("goal_tolerance", 3.0)  # m
        self.declare_parameter("stopped_speed", 0.5)  # m/s
        self.declare_parameter("results_dir", os.path.expanduser("~/av_results"))
        self.goal = None
        self.start_time = None
        self.last = None
        self.distance = 0.0
        self.max_speed = 0.0
        self.collisions = []
        self.done = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, "/goal_pose", lambda m: setattr(self, "goal", m), latched)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_odom, 10)
        self.create_subscription(Vector3Stamped, "/ground_truth/collision", self.on_collision, 10)

    def on_collision(self, msg: Vector3Stamped):
        self.collisions.append(msg.header.frame_id)
        self.finish("FAIL", f"collision with {msg.header.frame_id}")

    def on_odom(self, msg: Odometry):
        if self.done:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if self.start_time is None:
            self.start_time = t
        if self.last is not None:
            self.distance += math.hypot(p.x - self.last[0], p.y - self.last[1])
        self.last = (p.x, p.y)
        self.max_speed = max(self.max_speed, speed)
        self.elapsed = t - self.start_time

        if self.goal is not None:
            g = self.goal.pose.position
            to_goal = math.hypot(g.x - p.x, g.y - p.y)
            if (
                to_goal < self.get_parameter("goal_tolerance").value
                and speed < self.get_parameter("stopped_speed").value
            ):
                self.finish("PASS", f"stopped {to_goal:.1f} m from the goal")
                return
        if self.elapsed > self.get_parameter("timeout").value:
            self.finish("FAIL", "timeout")

    def finish(self, outcome: str, reason: str):
        if self.done:
            return
        self.done = True
        name = self.get_parameter("scenario_name").value
        result = {
            "scenario": name,
            "outcome": outcome,
            "reason": reason,
            "sim_time_s": round(getattr(self, "elapsed", 0.0), 2),
            "distance_m": round(self.distance, 1),
            "max_speed_mps": round(self.max_speed, 2),
            "collisions": self.collisions,
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        out_dir = self.get_parameter("results_dir").value
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.json")
        with open(path, "w") as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f"{outcome}: {name} ({reason}) -> {path}")


def main():
    rclpy.init()
    node = ScenarioMonitor()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
