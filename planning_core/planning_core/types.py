"""Core data types used by every algorithm. ROS nodes convert messages to these."""

from dataclasses import dataclass, field
from typing import List


@dataclass
class EgoState:
    x: float
    y: float
    yaw: float
    v: float
    stamp: float = 0.0


@dataclass
class TrajectoryPoint:
    x: float
    y: float
    yaw: float
    curvature: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0
    arc_length: float = 0.0
    time_from_start: float = 0.0


@dataclass
class Trajectory:
    points: List[TrajectoryPoint] = field(default_factory=list)
    stamp: float = 0.0
    sequence_id: int = 0
    behavior_state: int = 0
    emergency_stop: bool = False


@dataclass
class Obstacle:
    id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    length: float = 0.5
    width: float = 0.3
