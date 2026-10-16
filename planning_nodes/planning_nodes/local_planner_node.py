"""Local planner node (Member B, tasks P1.7 / P2.4).

In:  /planning/route, /planning/behavior, /world/ego_state, /world/costmap, /world/obstacles
Out: /planning/trajectory (av_msgs/Trajectory) at 10-20 Hz

Version 0 (this file): follows the route centerline with a curvature-aware velocity
profile and honours STOP / E_STOP. It already makes the full loop runnable.
TODO(member B): replace `plan_centerline` with lattice candidates + collision checks + cost.
"""

import numpy as np
import rclpy
from av_msgs.msg import BehaviorCommand, TrackedObjectArray
from av_msgs.msg import Trajectory as TrajectoryMsg
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from planning_core.geometry.path import arc_lengths, curvatures, headings
from planning_core.trajectory.velocity_profile import time_stamps, velocity_profile
from planning_core.types import Trajectory, TrajectoryPoint
from planning_core.vehicle import VehicleParams
from planning_nodes.conversions import odometry_to_ego, trajectory_to_msg


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__("local_planner")
        self.declare_parameter("vehicle_file", "")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("horizon_m", 8.0)
        self.declare_parameter("closed_loop", True)
        vehicle_file = self.get_parameter("vehicle_file").value
        if not vehicle_file:
            raise RuntimeError("parameter vehicle_file is required")
        self.vehicle = VehicleParams.from_yaml(vehicle_file)

        self.route_xy = None
        self.ego = None
        self.behavior = None
        self.costmap = None
        self.obstacles = None
        self.seq = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, "/planning/route", self.on_route, latched)
        self.create_subscription(Odometry, "/world/ego_state", lambda m: setattr(self, "ego", m), 10)
        self.create_subscription(
            BehaviorCommand, "/planning/behavior", lambda m: setattr(self, "behavior", m), 10
        )
        self.create_subscription(OccupancyGrid, "/world/costmap", lambda m: setattr(self, "costmap", m), 1)
        self.create_subscription(
            TrackedObjectArray, "/world/obstacles", lambda m: setattr(self, "obstacles", m), 10
        )
        self.pub = self.create_publisher(TrajectoryMsg, "/planning/trajectory", 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)

    def on_route(self, msg: Path):
        self.route_xy = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def window(self, ego):
        """Route points from the one nearest the car out to the horizon (wrapping on loops)."""
        xy = self.route_xy
        i0 = int(np.argmin(np.hypot(xy[:, 0] - ego.x, xy[:, 1] - ego.y)))
        if self.get_parameter("closed_loop").value:
            xy = np.vstack([xy[i0:], xy[:i0], xy[i0 : i0 + 1]])
        else:
            xy = xy[i0:]
        s = arc_lengths(xy)
        keep = s <= self.get_parameter("horizon_m").value
        keep[: min(2, len(keep))] = True
        return xy[keep]

    def plan_centerline(self, ego, state: int) -> Trajectory:
        xy = self.window(ego)
        if len(xy) < 2:
            return Trajectory(emergency_stop=True)
        s = arc_lengths(xy)
        k = np.clip(curvatures(xy), -self.vehicle.max_curvature, self.vehicle.max_curvature)
        v_limit = self.vehicle.max_speed
        if self.behavior is not None and self.behavior.speed_limit > 0.0:
            v_limit = min(v_limit, self.behavior.speed_limit)
        v_end = 0.0 if state == BehaviorCommand.STOP else None
        if state == BehaviorCommand.STOP:
            v_limit = min(v_limit, ego.v)  # never speed up while stopping
        v = velocity_profile(
            s,
            k,
            v_limit,
            self.vehicle.max_lateral_accel,
            self.vehicle.max_accel,
            self.vehicle.max_decel,
            ego.v,
            v_end,
        )
        t = time_stamps(s, v)
        yaw = headings(xy)
        acc = np.zeros_like(v)
        ds = np.diff(s)
        ok = ds > 1e-6
        acc[:-1][ok] = (v[1:] ** 2 - v[:-1] ** 2)[ok] / (2.0 * ds[ok])
        acc[-1] = acc[-2] if len(acc) > 1 else 0.0
        pts = [
            TrajectoryPoint(
                x=float(xy[i, 0]),
                y=float(xy[i, 1]),
                yaw=float(yaw[i]),
                curvature=float(k[i]),
                velocity=float(v[i]),
                acceleration=float(acc[i]),
                arc_length=float(s[i]),
                time_from_start=float(t[i]),
            )
            for i in range(len(xy))
        ]
        return Trajectory(points=pts, behavior_state=state)

    def tick(self):
        if self.route_xy is None or self.ego is None:
            return
        ego = odometry_to_ego(self.ego)
        state = self.behavior.state if self.behavior is not None else BehaviorCommand.FOLLOW_LANE
        if state == BehaviorCommand.E_STOP:
            traj = Trajectory(emergency_stop=True, behavior_state=state)
        else:
            traj = self.plan_centerline(ego, state)
        self.seq += 1
        traj.sequence_id = self.seq
        header = self.ego.header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.pub.publish(trajectory_to_msg(traj, header))


def main():
    rclpy.init()
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
