"""Polyline utilities: arc length, resampling, headings, curvature."""

import numpy as np


def arc_lengths(xy: np.ndarray) -> np.ndarray:
    """Cumulative distance along an (N, 2) polyline, starting at 0."""
    xy = np.asarray(xy, dtype=float)
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def resample(xy: np.ndarray, ds: float) -> np.ndarray:
    """Resample a polyline to (approximately) uniform spacing ds."""
    xy = np.asarray(xy, dtype=float)
    s = arc_lengths(xy)
    n = max(int(np.ceil(s[-1] / ds)) + 1, 2)
    s_new = np.linspace(0.0, s[-1], n)
    return np.column_stack([np.interp(s_new, s, xy[:, 0]), np.interp(s_new, s, xy[:, 1])])


def headings(xy: np.ndarray) -> np.ndarray:
    """Heading of each point (from central differences)."""
    xy = np.asarray(xy, dtype=float)
    d = np.gradient(xy, axis=0)
    return np.arctan2(d[:, 1], d[:, 0])


def curvatures(xy: np.ndarray) -> np.ndarray:
    """Signed curvature (left positive) of each point, 1/m."""
    xy = np.asarray(xy, dtype=float)
    yaw = np.unwrap(headings(xy))
    s = arc_lengths(xy)
    ds = np.gradient(s)
    ds[ds < 1e-9] = 1e-9
    return np.gradient(yaw) / ds
