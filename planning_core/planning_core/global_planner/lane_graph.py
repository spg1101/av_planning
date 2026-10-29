"""Road representation for the global planner.

Two sources, same class:
  CARLA:     a lane graph JSON exported once per town by `planning_carla export_lane_graph`
             (directed lanes, junction connections, lane changes).
  F1TENTH:   a waypoint loop CSV (x, y per line), e.g. recorded with `waypoint_recorder`.
"""

import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class LaneGraph:
    positions: Dict[int, Tuple[float, float]] = field(default_factory=dict)
    headings: Dict[int, float] = field(default_factory=dict)
    edges: Dict[int, List[Tuple[int, float]]] = field(default_factory=dict)
    closed_loop: bool = False

    @classmethod
    def from_file(cls, path: str, closed_loop: bool = False) -> "LaneGraph":
        if path.endswith(".json"):
            return cls.from_json(path)
        return cls.from_waypoints_csv(path, closed_loop=closed_loop)

    @classmethod
    def from_json(cls, path: str) -> "LaneGraph":
        with open(path) as f:
            data = json.load(f)
        graph = cls(closed_loop=False)
        for i, (x, y, yaw) in enumerate(data["nodes"]):
            graph.positions[i] = (float(x), float(y))
            graph.headings[i] = float(yaw)
            graph.edges[i] = []
        for a, b, cost in data["edges"]:
            graph.edges[int(a)].append((int(b), float(cost)))
        return graph

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
        for i in range(n):
            nx, ny = xy[(i + 1) % n] if (closed_loop or i < n - 1) else xy[i - 1]
            sign = 1.0 if (closed_loop or i < n - 1) else -1.0
            graph.headings[i] = math.atan2(sign * (ny - xy[i][1]), sign * (nx - xy[i][0]))
        last = n if closed_loop else n - 1
        for i in range(last):
            j = (i + 1) % n
            graph.edges[i].append((j, graph.distance(i, j)))
        return graph

    def distance(self, a: int, b: int) -> float:
        (xa, ya), (xb, yb) = self.positions[a], self.positions[b]
        return math.hypot(xb - xa, yb - ya)

    def nearest_node(
        self, x: float, y: float, yaw: Optional[float] = None, max_heading_error: float = 1.0
    ) -> int:
        """Closest node; with `yaw`, only nodes whose lane points the same way (within
        max_heading_error rad) are considered, so the car never snaps onto the opposite lane."""
        candidates = list(self.positions)
        if yaw is not None and self.headings:
            aligned = [k for k in candidates if abs(_wrap(self.headings[k] - yaw)) <= max_heading_error]
            candidates = aligned or candidates
        return min(candidates, key=lambda k: math.hypot(self.positions[k][0] - x, self.positions[k][1] - y))

    def ordered_xy(self) -> np.ndarray:
        return np.array([self.positions[k] for k in sorted(self.positions)])

    def path_xy(self, nodes: List[int]) -> np.ndarray:
        return np.array([self.positions[k] for k in nodes])


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))
