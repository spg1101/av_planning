"""The ONLY place that knows both ROS messages and planning_core types.

Nodes call these helpers so algorithms never touch messages directly.
"""

import math

from av_msgs.msg import Trajectory as TrajectoryMsg
from av_msgs.msg import TrajectoryPoint as TrajectoryPointMsg
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from planning_core.types import EgoState, Obstacle, Trajectory, TrajectoryPoint


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def seconds_to_duration(t: float) -> Duration:
    sec = int(math.floor(t))
    nanosec = int(round((t - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec, nanosec = sec + 1, nanosec - 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def duration_to_seconds(d: Duration) -> float:
    return d.sec + d.nanosec * 1e-9


def stamp_to_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def odometry_to_ego(msg: Odometry) -> EgoState:
    p = msg.pose.pose
    return EgoState(
        x=p.position.x,
        y=p.position.y,
        yaw=quaternion_to_yaw(p.orientation),
        v=math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
        stamp=stamp_to_seconds(msg.header.stamp),
    )


def trajectory_to_msg(traj: Trajectory, header) -> TrajectoryMsg:
    msg = TrajectoryMsg()
    msg.header = header
    msg.sequence_id = traj.sequence_id
    msg.behavior_state = int(traj.behavior_state)
    msg.emergency_stop = traj.emergency_stop
    for p in traj.points:
        m = TrajectoryPointMsg()
        m.pose.position.x = p.x
        m.pose.position.y = p.y
        m.pose.orientation = yaw_to_quaternion(p.yaw)
        m.curvature = float(p.curvature)
        m.velocity = float(p.velocity)
        m.acceleration = float(p.acceleration)
        m.arc_length = float(p.arc_length)
        m.time_from_start = seconds_to_duration(p.time_from_start)
        msg.points.append(m)
    return msg


def msg_to_trajectory(msg: TrajectoryMsg) -> Trajectory:
    pts = [
        TrajectoryPoint(
            x=m.pose.position.x,
            y=m.pose.position.y,
            yaw=quaternion_to_yaw(m.pose.orientation),
            curvature=m.curvature,
            velocity=m.velocity,
            acceleration=m.acceleration,
            arc_length=m.arc_length,
            time_from_start=duration_to_seconds(m.time_from_start),
        )
        for m in msg.points
    ]
    return Trajectory(
        points=pts,
        stamp=stamp_to_seconds(msg.header.stamp),
        sequence_id=msg.sequence_id,
        behavior_state=msg.behavior_state,
        emergency_stop=msg.emergency_stop,
    )


def tracked_objects_to_obstacles(msg) -> list:
    return [
        Obstacle(
            id=o.id,
            x=o.pose.position.x,
            y=o.pose.position.y,
            vx=o.velocity.x,
            vy=o.velocity.y,
            length=o.size.x,
            width=o.size.y,
        )
        for o in msg.objects
    ]
