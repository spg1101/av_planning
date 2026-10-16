"""Global planner node (Member B, task P1.5).

In:  /goal_pose (PoseStamped), /world/ego_state (Odometry)
Out: /planning/route (nav_msgs/Path, latched)
On a closed loop with no goal, it publishes the whole loop once at startup.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from planning_core.global_planner.astar import astar
from planning_core.global_planner.lane_graph import LaneGraph
from planning_nodes.conversions import yaw_to_quaternion


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("global_planner")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("closed_loop", True)
        self.declare_parameter("frame_id", "map")
        path = self.get_parameter("waypoints_file").value
        closed = self.get_parameter("closed_loop").value
        self.frame_id = self.get_parameter("frame_id").value
        if not path:
            raise RuntimeError("parameter waypoints_file is required")
        self.graph = LaneGraph.from_waypoints_csv(path, closed_loop=closed)
        self.ego = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route_pub = self.create_publisher(Path, "/planning/route", latched)
        self.create_subscription(Odometry, "/world/ego_state", self.on_ego, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)

        if closed:
            self.publish_nodes(sorted(self.graph.positions))
        self.get_logger().info(f"loaded {len(self.graph.positions)} waypoints from {path}")

    def on_ego(self, msg: Odometry):
        self.ego = msg

    def on_goal(self, msg: PoseStamped):
        if self.ego is None:
            self.get_logger().warn("goal received before ego state; ignoring")
            return
        p = self.ego.pose.pose.position
        start = self.graph.nearest_node(p.x, p.y)
        goal = self.graph.nearest_node(msg.pose.position.x, msg.pose.position.y)
        g = self.graph
        nodes = astar(g.edges, start, goal, heuristic=lambda a, b: g.distance(a, b))
        if nodes is None:
            self.get_logger().error("no route to goal")
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
            pose.pose.orientation = yaw_to_quaternion(math.atan2(ny - y, nx - x))
            path.poses.append(pose)
        self.route_pub.publish(path)
        self.get_logger().info(f"published route with {len(nodes)} poses")


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
