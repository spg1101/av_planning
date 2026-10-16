"""Generic A* over a weighted directed graph."""

import heapq
import itertools
from typing import Callable, Dict, Hashable, List, Optional, Tuple

Graph = Dict[Hashable, List[Tuple[Hashable, float]]]


def astar(
    graph: Graph,
    start: Hashable,
    goal: Hashable,
    heuristic: Callable[[Hashable, Hashable], float] = lambda a, b: 0.0,
) -> Optional[List[Hashable]]:
    """Return the lowest-cost node list from start to goal, or None if unreachable."""
    counter = itertools.count()
    open_set = [(heuristic(start, goal), next(counter), start)]
    came_from: Dict[Hashable, Hashable] = {}
    g = {start: 0.0}
    closed = set()
    while open_set:
        _, _, node = heapq.heappop(open_set)
        if node == goal:
            path = [node]
            while node in came_from:
                node = came_from[node]
                path.append(node)
            return path[::-1]
        if node in closed:
            continue
        closed.add(node)
        for nbr, cost in graph.get(node, []):
            new_g = g[node] + cost
            if new_g < g.get(nbr, float("inf")):
                g[nbr] = new_g
                came_from[nbr] = node
                heapq.heappush(open_set, (new_g + heuristic(nbr, goal), next(counter), nbr))
    return None
