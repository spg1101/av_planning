"""Canned trajectories for the controls team to tune against (Lead, task P1.8).

Parameter `shape`: straight | circle | lane_change | stop | speed_ramp
Starts at the car's position when the node starts (TF map->base_link), published at 10 Hz.
"""

import math

import numpy as np
import rclpy
from av_msgs.msg import Trajectory as TrajectoryMsg
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from planning_core.geometry.path import arc_lengths, curvatures, headings
from planning_core.trajectory.velocity_profile import time_stamps
from planning_core.types import Trajectory, TrajectoryPoint
from planning_nodes.conversions import quaternion_to_yaw, trajectory_to_msg


def make_shape(shape: str, speed: float):
    s = np.arange(0.0, 30.0, 0.1)
    if shape == "circle":
        r = 3.0
        xy = np.column_stack([r * np.sin(s / r), r * (1 - np.cos(s / r))])
        v = np.full_like(s, speed)
    elif shape == "lane_change":
        y = 0.6 / (1 + np.exp(-(s - 10.0)))
        xy = np.column_stack([s, y])
        v = np.full_like(s, speed)
    elif shape == "stop":
        xy = np.column_stack([s, np.zeros_like(s)])
        v = np.minimum(speed, np.sqrt(np.maximum(2 * 1.0 * (20.0 - s), 0.0)))
    elif shape == "speed_ramp":
        xy = np.column_stack([s, np.zeros_like(s)])
        v = np.minimum(speed, 0.5 + 0.1 * s)
    else:  # straight
        xy = np.column_stack([s, np.zeros_like(s)])
        v = np.full_like(s, speed)
    return xy, v


class CannedTrajectory(Node):
    def __init__(self):
        super().__init__("canned_trajectory")
        self.declare_parameter("shape", "straight")
        self.declare_parameter("speed", 2.0)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(TrajectoryMsg, "/planning/trajectory", 10)
        self.traj, self.start_time, self.seq = None, None, 0
        self.create_timer(0.1, self.tick)

    def build(self, x0, y0, yaw0):
        xy, v = make_shape(self.get_parameter("shape").value, self.get_parameter("speed").value)
        c, s_ = math.cos(yaw0), math.sin(yaw0)
        xy = np.column_stack([x0 + c * xy[:, 0] - s_ * xy[:, 1], y0 + s_ * xy[:, 0] + c * xy[:, 1]])
        s, k, yaw = arc_lengths(xy), curvatures(xy), headings(xy)
        t = time_stamps(s, v)
        pts = [
            TrajectoryPoint(
                x=float(xy[i, 0]),
                y=float(xy[i, 1]),
                yaw=float(yaw[i]),
                curvature=float(k[i]),
                velocity=float(v[i]),
                arc_length=float(s[i]),
                time_from_start=float(t[i]),
            )
            for i in range(len(s))
        ]
        return Trajectory(points=pts)

    def tick(self):
        if self.traj is None:
            try:
                tf = self.tf_buffer.lookup_transform("map", "base_link", Time())
            except TransformException:
                return
            self.traj = self.build(
                tf.transform.translation.x,
                tf.transform.translation.y,
                quaternion_to_yaw(tf.transform.rotation),
            )
            self.start_time = self.get_clock().now()
        self.seq += 1
        self.traj.sequence_id = self.seq
        msg = trajectory_to_msg(self.traj, TrajectoryMsg().header)
        msg.header.frame_id = "map"
        msg.header.stamp = self.start_time.to_msg()  # one plan: time-index from its start
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CannedTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
