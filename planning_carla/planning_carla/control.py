"""AckermannDrive (speed m/s, steering rad) -> CARLA VehicleControl (throttle, brake, steer).

Pure math so it can be unit-tested. This conversion belongs to the platform adapter: the
planner and the controls team always speak speed + steering angle, on every platform.
"""

from dataclasses import dataclass


@dataclass
class LongitudinalGains:
    kp: float = 0.5
    ki: float = 0.05
    kb: float = 0.3  # brake gain per m/s of overspeed
    deadband: float = 0.2  # m/s of overspeed tolerated before braking
    hold_brake_speed: float = 0.3  # below this, with a zero command, hold the brake


class AckermannToCarla:
    def __init__(self, max_steer_rad: float, gains: LongitudinalGains = LongitudinalGains()):
        if max_steer_rad <= 0.0:
            raise ValueError("max_steer_rad must be positive")
        self.max_steer = max_steer_rad
        self.g = gains
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def __call__(self, target_speed: float, steering_angle: float, current_speed: float, dt: float):
        """Return (throttle, brake, steer), each already in CARLA's range.

        CARLA steer is +1 for a full RIGHT turn; ROS steering angles are positive LEFT.
        """
        steer = max(-1.0, min(1.0, -steering_angle / self.max_steer))
        target_speed = max(0.0, target_speed)  # reverse is not part of the ICD

        if target_speed < 1e-3 and current_speed < self.g.hold_brake_speed:
            self.integral = 0.0
            return 0.0, 1.0, steer

        error = target_speed - current_speed
        if error < -self.g.deadband:
            self.integral = 0.0
            return 0.0, min(1.0, self.g.kb * (-error)), steer

        self.integral = max(-2.0, min(2.0, self.integral + error * dt))
        throttle = self.g.kp * error + self.g.ki * self.integral
        return max(0.0, min(1.0, throttle)), 0.0, steer
