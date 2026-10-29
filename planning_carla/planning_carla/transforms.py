"""CARLA <-> ROS coordinate conversions. Pure math: no carla or rclpy imports, so it is unit-tested.

CARLA (Unreal) is LEFT-handed: x forward, y RIGHT, z up, angles in degrees, yaw clockwise.
ROS (REP 103) is RIGHT-handed: x forward, y LEFT, z up, angles in radians, yaw counter-clockwise.

So: flip y, and flip the sign of pitch and yaw (roll keeps its sign). These are the same
conventions the official carla_ros_bridge uses, so data matches other CARLA/ROS tools.
"""

import math
from typing import Tuple

import numpy as np


def location_to_ros(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return x, -y, z


def rotation_to_ros(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float]:
    """Return (roll, pitch, yaw) in radians, ROS convention."""
    return math.radians(roll_deg), -math.radians(pitch_deg), -math.radians(yaw_deg)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def world_velocity_to_body(vx: float, vy: float, ros_yaw: float) -> Tuple[float, float]:
    """CARLA world-frame velocity (m/s) -> ROS body-frame (forward, left) velocity."""
    rx, ry = vx, -vy
    c, s = math.cos(ros_yaw), math.sin(ros_yaw)
    return c * rx + s * ry, -s * rx + c * ry


def yaw_rate_to_ros(angular_z_deg_s: float) -> float:
    """CARLA angular velocity z (deg/s, clockwise) -> ROS yaw rate (rad/s, counter-clockwise)."""
    return -math.radians(angular_z_deg_s)


def lidar_to_ros(raw: np.ndarray) -> np.ndarray:
    """CARLA lidar buffer (N, 4) float32 [x, y, z, intensity] -> same layout in the ROS frame."""
    pts = np.array(raw, dtype=np.float32).reshape(-1, 4).copy()
    pts[:, 1] *= -1.0
    return pts


def imu_to_ros(accel: Tuple[float, float, float], gyro: Tuple[float, float, float]):
    """CARLA IMU -> ROS. Returns ((ax, ay, az) m/s^2, (wx, wy, wz) rad/s).

    CARLA gyroscope is already in rad/s; only the handedness changes.
    """
    ax, ay, az = accel
    gx, gy, gz = gyro
    return (ax, -ay, az), (-gx, gy, -gz)


def rear_axle_pose(x: float, y: float, yaw: float, rear_axle_offset: float) -> Tuple[float, float]:
    """Shift a ROS-frame vehicle origin back to the rear axle (base_link).

    rear_axle_offset: signed distance from the actor origin to the rear axle along the
    vehicle's forward axis (negative: the rear axle is behind the origin).
    """
    return x + rear_axle_offset * math.cos(yaw), y + rear_axle_offset * math.sin(yaw)
