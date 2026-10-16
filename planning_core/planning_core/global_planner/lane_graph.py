"""Road representation for the global planner.

F1TENTH: a waypoint loop from a CSV (x, y per line).
CARLA (Phase 3): export the OpenDRIVE topology once to a file and load it here.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np


@dataclass
class LaneGraph:
    positions: Dict[int, Tuple[float, float]] = field(default_factory=dict)
    edges: Dict[int, List[Tuple[int, float]]] = field(default_factory=dict)
    closed_loop: bool = False

    @classmethod
    def from_waypoints_csv(cls, path: str, closed_loop: bool = True) -> "LaneGraph":
        xy = np.loadtxt(path, delimiter=",", comments="#", usecols=(0, 1), ndmin=2)
        return cls.from_waypoints(xy, closed_loop)

    @classmethod
    def from_waypoints(cls, xy, closed_loop: bool = True) -> "LaneGraph":
        graph = cls(closed_loop=closed_loop)
        n = len(xy)
        for i, (x, y) in enumerate(xy):
            graph.positions[i] = (float(x), float(y))
            graph.edges[i] = []
        last = n if closed_loop else n - 1
        for i in range(last):
            j = (i + 1) % n
            graph.edges[i].append((j, graph.distance(i, j)))
        return graph

    def distance(self, a: int, b: int) -> float:
        (xa, ya), (xb, yb) = self.positions[a], self.positions[b]
        return math.hypot(xb - xa, yb - ya)

    def nearest_node(self, x: float, y: float) -> int:
        return min(
            self.positions, key=lambda k: math.hypot(self.positions[k][0] - x, self.positions[k][1] - y)
        )

    def ordered_xy(self) -> np.ndarray:
        return np.array([self.positions[k] for k in sorted(self.positions)])
