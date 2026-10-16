"""Candidate cost function (Member B). Lower is better."""

from dataclasses import dataclass

import numpy as np

from planning_core.geometry.path import curvatures


@dataclass
class CostWeights:
    lateral_offset: float = 1.0
    curvature: float = 0.5
    lane_change: float = 2.0


def candidate_cost(
    xy: np.ndarray, final_offset: float, target_offset: float, weights: CostWeights = CostWeights()
) -> float:
    k = curvatures(xy)
    return float(
        weights.lateral_offset * abs(final_offset)
        + weights.curvature * np.mean(np.abs(k))
        + weights.lane_change * abs(final_offset - target_offset)
    )
