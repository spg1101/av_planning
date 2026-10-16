"""Frenet frame (s, d) relative to a polyline reference path.

s = distance along the reference, d = signed lateral offset (left positive).
Both conversions use the same segment geometry, so a round trip is exact up to
floating point except right at segment joints.
"""

from typing import Tuple

import numpy as np

from planning_core.geometry.path import arc_lengths


class ReferencePath:
    def __init__(self, xy: np.ndarray):
        self.xy = np.asarray(xy, dtype=float)
        if len(self.xy) < 2:
            raise ValueError("reference path needs at least 2 points")
        self.s = arc_lengths(self.xy)
        d = np.diff(self.xy, axis=0)
        self.seg_len = np.linalg.norm(d, axis=1)
        self.seg_yaw = np.arctan2(d[:, 1], d[:, 0])

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def to_frenet(self, x: float, y: float) -> Tuple[float, float]:
        p = np.array([x, y])
        a = self.xy[:-1]
        ab = self.xy[1:] - a
        t = np.einsum("ij,ij->i", p - a, ab) / np.maximum(self.seg_len**2, 1e-12)
        t = np.clip(t, 0.0, 1.0)
        proj = a + ab * t[:, None]
        dist = np.linalg.norm(proj - p, axis=1)
        i = int(np.argmin(dist))
        s = self.s[i] + t[i] * self.seg_len[i]
        normal = np.array([-np.sin(self.seg_yaw[i]), np.cos(self.seg_yaw[i])])
        d = float(np.dot(p - proj[i], normal))
        return float(s), d

    def to_cartesian(self, s: float, d: float) -> Tuple[float, float, float]:
        """Return (x, y, yaw of the reference at s)."""
        s = float(np.clip(s, 0.0, self.length))
        i = int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.seg_len) - 1))
        t = (s - self.s[i]) / max(self.seg_len[i], 1e-12)
        base = self.xy[i] + t * (self.xy[i + 1] - self.xy[i])
        yaw = self.seg_yaw[i]
        x = base[0] - d * np.sin(yaw)
        y = base[1] + d * np.cos(yaw)
        return float(x), float(y), float(yaw)
