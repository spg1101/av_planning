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


def test_json_graph_and_heading_aware_nearest(tmp_path):
    import json

    # two parallel lanes in opposite directions, 3.5 m apart
    nodes = [[0, 0, 0.0], [10, 0, 0.0], [10, 3.5, 3.1416], [0, 3.5, 3.1416]]
    edges = [[0, 1, 10.0], [2, 3, 10.0]]
    path = tmp_path / "g.json"
    path.write_text(json.dumps({"frame": "map", "nodes": nodes, "edges": edges}))
    g = LaneGraph.from_file(str(path))
    assert astar(g.edges, 0, 1) == [0, 1]
    # a car at y=1.9 (closer to the upper lane) heading +x must snap to the lower, same-direction lane
    assert g.nearest_node(1.0, 1.9, yaw=0.0) == 0
    assert g.nearest_node(1.0, 1.9) == 3
