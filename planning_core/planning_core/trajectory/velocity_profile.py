"""Speed profile along a path respecting curvature, acceleration and braking limits."""

from typing import Optional

import numpy as np


def velocity_profile(
    s: np.ndarray,
    curvature: np.ndarray,
    v_max: float,
    a_lat_max: float,
    a_accel: float,
    a_decel: float,
    v_start: float,
    v_end: Optional[float] = None,
) -> np.ndarray:
    """Forward-backward pass.

    1. Cap speed by v_max and by lateral acceleration: v <= sqrt(a_lat_max / |k|).
    2. Forward pass: speed can only increase by a_accel from v_start.
    3. Backward pass: speed must be able to brake (a_decel) to v_end / later caps.
    """
    s = np.asarray(s, dtype=float)
    k = np.abs(np.asarray(curvature, dtype=float))
    with np.errstate(divide="ignore"):
        v_curve = np.where(k > 1e-6, np.sqrt(a_lat_max / np.maximum(k, 1e-6)), np.inf)
    v = np.minimum(v_max, v_curve)
    if v_end is not None:
        v[-1] = min(v[-1], v_end)

    v[0] = min(v[0], max(v_start, 0.0))
    for i in range(1, len(v)):
        ds = s[i] - s[i - 1]
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2.0 * a_accel * ds))
    for i in range(len(v) - 2, -1, -1):
        ds = s[i + 1] - s[i]
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2.0 * a_decel * ds))
    return v


def time_stamps(s: np.ndarray, v: np.ndarray, v_floor: float = 0.05) -> np.ndarray:
    """Time to reach each point, integrating ds / v_avg. Stopped segments add no time."""
    t = np.zeros(len(s))
    for i in range(1, len(s)):
        v_avg = 0.5 * (v[i] + v[i - 1])
        t[i] = t[i - 1] + ((s[i] - s[i - 1]) / v_avg if v_avg > v_floor else 0.0)
    return t
