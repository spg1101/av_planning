"""Trajectory stitching: start each new plan from where the previous plan said the car
would be now, so replans don't jump. Fall back to the measured state if the car drifted.
"""

import math
from typing import Optional, Tuple

from planning_core.types import EgoState, Trajectory, TrajectoryPoint


def stitch_start(
    prev: Optional[Trajectory], now: float, ego: EgoState, max_deviation: float = 0.5
) -> Tuple[TrajectoryPoint, bool]:
    """Return (start point for the new plan, True if taken from the previous plan)."""
    measured = TrajectoryPoint(x=ego.x, y=ego.y, yaw=ego.yaw, velocity=ego.v)
    if prev is None or prev.emergency_stop or not prev.points:
        return measured, False
    elapsed = now - prev.stamp
    if elapsed < 0.0 or elapsed > prev.points[-1].time_from_start:
        return measured, False
    candidate = min(prev.points, key=lambda p: abs(p.time_from_start - elapsed))
    if math.hypot(candidate.x - ego.x, candidate.y - ego.y) > max_deviation:
        return measured, False
    return candidate, True
