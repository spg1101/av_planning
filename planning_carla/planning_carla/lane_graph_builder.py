"""Build a directed lane graph from sampled lane segments. Pure Python, unit-tested.

The CARLA-specific tool (tools/export_lane_graph.py) samples each lane segment of the
OpenDRIVE map into points and calls `build_lane_graph`. The result is saved once as JSON and
loaded by the global planner, so planning never needs a live CARLA connection to route.

JSON format (map frame = CARLA world with y flipped, i.e. ROS convention):
    {"frame": "map", "nodes": [[x, y, yaw], ...], "edges": [[from, to, cost], ...]}
"""

import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

Key = Tuple[int, int, int]  # (road_id, section_id, lane_id)


@dataclass
class LaneSegment:
    key: Key
    points: List[Tuple[float, float, float]]  # (x, y, yaw) in driving order, ROS frame
    s: List[float]  # OpenDRIVE s of each point
    left_key: Optional[Key] = None  # neighbour lane, same driving direction
    right_key: Optional[Key] = None
    can_change_left: bool = False
    can_change_right: bool = False

    @property
    def direction(self) -> int:
        """+1 if s increases in the driving direction (lane_id < 0, right-hand traffic)."""
        return 1 if self.key[2] < 0 else -1


@dataclass
class LaneGraphData:
    nodes: List[Tuple[float, float, float]] = field(default_factory=list)
    edges: List[Tuple[int, int, float]] = field(default_factory=list)

    def to_json(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(
                {
                    "frame": "map",
                    "nodes": [[round(x, 3), round(y, 3), round(t, 4)] for x, y, t in self.nodes],
                    "edges": [[a, b, round(c, 3)] for a, b, c in self.edges],
                },
                f,
            )


def build_lane_graph(
    segments: List[LaneSegment],
    connect_tolerance: float = 0.5,
    lane_change_distance: float = 10.0,
    lane_change_penalty: float = 2.0,
) -> LaneGraphData:
    graph = LaneGraphData()
    seg_nodes: List[List[int]] = []
    by_key: Dict[Key, int] = {}

    # 1. Nodes and edges along each lane.
    for si, seg in enumerate(segments):
        ids = []
        for x, y, yaw in seg.points:
            ids.append(len(graph.nodes))
            graph.nodes.append((x, y, yaw))
        for a, b in zip(ids[:-1], ids[1:], strict=True):
            graph.edges.append((a, b, _dist(graph.nodes[a], graph.nodes[b])))
        seg_nodes.append(ids)
        by_key[seg.key] = si

    # 2. Successor links: a segment's last point touches another segment's first point.
    starts = [(si, graph.nodes[ids[0]]) for si, ids in enumerate(seg_nodes) if ids]
    for si, ids in enumerate(seg_nodes):
        if not ids:
            continue
        end = graph.nodes[ids[-1]]
        for sj, start in starts:
            if sj != si and _dist(end, start) <= connect_tolerance:
                graph.edges.append((ids[-1], seg_nodes[sj][0], _dist(end, start)))

    # 3. Lane changes into the neighbouring lane, landing lane_change_distance ahead.
    for si, seg in enumerate(segments):
        for allowed, nkey in ((seg.can_change_left, seg.left_key), (seg.can_change_right, seg.right_key)):
            if not allowed or nkey is None or nkey not in by_key:
                continue
            nseg = segments[by_key[nkey]]
            if nseg.direction != seg.direction:
                continue
            for k, node in enumerate(seg_nodes[si]):
                target_progress = seg.direction * seg.s[k] + lane_change_distance
                best = _closest_progress(nseg, target_progress)
                if best is None:
                    continue
                j = seg_nodes[by_key[nkey]][best]
                d = _dist(graph.nodes[node], graph.nodes[j])
                graph.edges.append((node, j, d * lane_change_penalty))
    return graph


def _closest_progress(seg: LaneSegment, target: float) -> Optional[int]:
    if not seg.s:
        return None
    progress = [seg.direction * s for s in seg.s]
    k = min(range(len(progress)), key=lambda i: abs(progress[i] - target))
    spacing = abs(progress[-1] - progress[0]) / max(len(progress) - 1, 1)
    return k if abs(progress[k] - target) <= max(spacing, 1.0) else None


def _dist(a, b) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])
