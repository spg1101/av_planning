import math

import numpy as np
import pytest

from planning_carla.control import AckermannToCarla
from planning_carla.transforms import (
    euler_to_quaternion,
    imu_to_ros,
    lidar_to_ros,
    location_to_ros,
    rear_axle_pose,
    rotation_to_ros,
    world_velocity_to_body,
    yaw_rate_to_ros,
)


def test_y_and_yaw_flip():
    assert location_to_ros(1.0, 2.0, 3.0) == (1.0, -2.0, 3.0)
    # CARLA yaw +90 deg points along CARLA +y (to the right), which is ROS -y: yaw -pi/2.
    _, _, yaw = rotation_to_ros(0.0, 0.0, 90.0)
    assert yaw == pytest.approx(-math.pi / 2)


def test_body_velocity_driving_forward():
    # Car heading along CARLA +y at 5 m/s -> ROS yaw -pi/2, all speed is forward.
    _, _, yaw = rotation_to_ros(0.0, 0.0, 90.0)
    fwd, left = world_velocity_to_body(0.0, 5.0, yaw)
    assert fwd == pytest.approx(5.0) and left == pytest.approx(0.0, abs=1e-9)


def test_left_turn_is_positive_yaw_rate():
    # Turning left in CARLA means yaw decreasing (counter-clockwise from above): negative z rate.
    assert yaw_rate_to_ros(-10.0) > 0.0


def test_lidar_and_imu_flip_y():
    pts = lidar_to_ros(np.array([[1.0, 2.0, 3.0, 0.5]], dtype=np.float32))
    assert pts.tolist() == [[1.0, -2.0, 3.0, 0.5]]
    a, g = imu_to_ros((1.0, 2.0, 3.0), (0.1, 0.2, 0.3))
    assert a == (1.0, -2.0, 3.0) and g == (-0.1, 0.2, -0.3)


def test_quaternion_yaw_only():
    x, y, z, w = euler_to_quaternion(0.0, 0.0, math.pi / 2)
    assert (x, y) == (0.0, 0.0) and z == pytest.approx(math.sqrt(0.5)) and w == pytest.approx(math.sqrt(0.5))


def test_rear_axle_shift():
    x, y = rear_axle_pose(10.0, 0.0, math.pi / 2, -1.4)
    assert x == pytest.approx(10.0) and y == pytest.approx(-1.4)


def test_control_signs_and_limits():
    ctl = AckermannToCarla(max_steer_rad=0.6)
    throttle, brake, steer = ctl(5.0, 0.3, 0.0, 0.05)
    assert throttle > 0.0 and brake == 0.0
    assert steer == pytest.approx(-0.5)  # left steering angle -> negative CARLA steer
    _, _, steer = ctl(5.0, -2.0, 0.0, 0.05)
    assert steer == 1.0  # clipped


def test_control_brakes_when_too_fast_and_holds_at_stop():
    ctl = AckermannToCarla(max_steer_rad=0.6)
    throttle, brake, _ = ctl(2.0, 0.0, 6.0, 0.05)
    assert throttle == 0.0 and brake > 0.0
    throttle, brake, _ = ctl(0.0, 0.0, 0.1, 0.05)
    assert (throttle, brake) == (0.0, 1.0)
