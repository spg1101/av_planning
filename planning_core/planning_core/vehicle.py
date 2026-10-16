"""Vehicle parameters shared by planning and controls (one YAML per platform)."""

import math
from dataclasses import dataclass

import yaml


@dataclass
class VehicleParams:
    wheelbase: float
    max_steer: float  # rad
    max_speed: float  # m/s
    max_accel: float  # m/s^2
    max_decel: float  # m/s^2, positive number
    max_lateral_accel: float  # m/s^2
    width: float
    length: float

    @property
    def max_curvature(self) -> float:
        return math.tan(self.max_steer) / self.wheelbase

    @classmethod
    def from_yaml(cls, path: str) -> "VehicleParams":
        with open(path) as f:
            data = yaml.safe_load(f)
        return cls(**data["vehicle"])
