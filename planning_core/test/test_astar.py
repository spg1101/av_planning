from planning_core.global_planner.astar import astar
from planning_core.global_planner.lane_graph import LaneGraph


def test_shortest_path():
    graph = {"a": [("b", 1.0), ("c", 5.0)], "b": [("c", 1.0)], "c": []}
    assert astar(graph, "a", "c") == ["a", "b", "c"]


def test_unreachable():
    assert astar({"a": [], "b": []}, "a", "b") is None


def test_loop_wraps_around():
    g = LaneGraph.from_waypoints([(0, 0), (1, 0), (1, 1), (0, 1)], closed_loop=True)
    assert astar(g.edges, 3, 1) == [3, 0, 1]
