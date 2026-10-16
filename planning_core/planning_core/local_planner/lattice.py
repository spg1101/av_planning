"""Frenet lattice candidate generation (Member B, task P1.7).

Each candidate moves from the current lateral offset d0 to a target offset d1 over a
longitudinal distance, using a quintic polynomial so lateral velocity and acceleration
start and end at zero.
"""

from typing import List

import numpy as np

from planning_core.geometry.frenet import ReferencePath


def quintic_lateral(d0: float, d1: float, length: float, s: np.ndarray) -> np.ndarray:
    """Lateral offset along s for a smooth d0 -> d1 transition over `length` metres."""
    tau = np.clip(s / max(length, 1e-6), 0.0, 1.0)
    blend = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    return d0 + (d1 - d0) * blend


def generate_candidates(
    ref: ReferencePath,
    s0: float,
    d0: float,
    horizon: float,
    target_offsets: List[float],
    transition_lengths: List[float],
    ds: float = 0.2,
) -> List[np.ndarray]:
    """Return candidate paths as (N, 2) xy arrays."""
    s_local = np.arange(0.0, horizon + ds, ds)
    candidates = []
    for d1 in target_offsets:
        for length in transition_lengths:
            d = quintic_lateral(d0, d1, length, s_local)
            xy = np.array([ref.to_cartesian(s0 + si, di)[:2] for si, di in zip(s_local, d, strict=True)])
            candidates.append(xy)
    return candidates
