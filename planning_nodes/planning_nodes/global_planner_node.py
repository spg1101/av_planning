"""Global planner node (Member B, task P1.5).

In:  /goal_pose (PoseStamped, latched: the CARLA adapter publishes the scenario goal),
     /world/ego_state (Odometry)
Out: /planning/route (nav_msgs/Path, latched)

graph_file: a lane graph JSON (CARLA, from export_lane_graph) or a waypoint CSV (loop track).
On a closed loop with no goal, it publishes the whole loop once at startup.
A goal that arrives before the first ego state is kept and planned as soon as the pose arrives.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from planning_core.global_planner.astar import astar
from planning_core.global_planner.lane_graph import LaneGraph
from planning_nodes.conversions import quaternion_to_yaw, yaw_to_quaternion


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("global_planner")
        self.declare_parameter("graph_file", "")
        self.declare_parameter("closed_loop", False)
        self.declare_parameter("frame_id", "map")
        path = self.get_parameter("graph_file").value
        closed = self.get_parameter("closed_loop").value
        self.frame_id = self.get_parameter("frame_id").value
        if not path:
            raise RuntimeError("parameter graph_file is required (lane graph .json or waypoint .csv)")
        self.graph = LaneGraph.from_file(path, closed_loop=closed)
        self.ego = None
        self.pending_goal = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_pub = self.create_publisher(Path, "/planning/route", latched)
        self.create_subscription(Odometry, "/world/ego_state", self.on_ego, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, latched)

        if closed:
            self.publish_nodes(sorted(self.graph.positions))
        self.get_logger().info(f"loaded {len(self.graph.positions)} nodes from {path}")

    def on_ego(self, msg: Odometry):
        self.ego = msg
        if self.pending_goal is not None:
            goal, self.pending_goal = self.pending_goal, None
            self.on_goal(goal)

    def on_goal(self, msg: PoseStamped):
        if self.ego is None:
            self.pending_goal = msg
            return
        p = self.ego.pose.pose
        g = self.graph
        start = g.nearest_node(p.position.x, p.position.y, yaw=quaternion_to_yaw(p.orientation))
        goal = g.nearest_node(
            msg.pose.position.x, msg.pose.position.y, yaw=quaternion_to_yaw(msg.pose.orientation)
        )
        nodes = astar(g.edges, start, goal, heuristic=lambda a, b: g.distance(a, b))
        if nodes is None:
            self.get_logger().error(f"no route from node {start} to node {goal}")
            return
        self.publish_nodes(nodes)

    def publish_nodes(self, nodes):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for i, n in enumerate(nodes):
            x, y = self.graph.positions[n]
            nx, ny = self.graph.positions[nodes[min(i + 1, len(nodes) - 1)]]
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            yaw = math.atan2(ny - y, nx - x) if (nx, ny) != (x, y) else self.graph.headings.get(n, 0.0)
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)
        self.route_pub.publish(path)
        length = sum(self.graph.distance(a, b) for a, b in zip(nodes[:-1], nodes[1:], strict=True))
        self.get_logger().info(f"published route: {len(nodes)} poses, {length:.0f} m")


def main():
    rclpy.init()
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
