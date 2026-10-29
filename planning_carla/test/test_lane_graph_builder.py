import json

from planning_carla.lane_graph_builder import LaneSegment, build_lane_graph


def straight_lane(lane_id, y, x0=0.0, length=40.0, step=2.0):
    n = int(length / step) + 1
    xs = [x0 + i * step for i in range(n)]
    return LaneSegment(key=(1, 0, lane_id), points=[(x, y, 0.0) for x in xs], s=list(xs))


def test_successor_link_and_lane_changes(tmp_path):
    right = straight_lane(-1, 0.0)
    left = straight_lane(-2, 3.5)
    right.left_key, right.can_change_left = left.key, True
    left.right_key, left.can_change_right = right.key, True
    after = LaneSegment(key=(2, 0, -1), points=[(40.0, 0.0, 0.0), (42.0, 0.0, 0.0)], s=[0.0, 2.0])

    g = build_lane_graph([right, left, after], lane_change_distance=10.0)
    n_right = len(right.points)
    n_left = len(left.points)
    edges = {(a, b) for a, b, _ in g.edges}

    # end of the right lane continues into the next road
    assert (n_right - 1, n_right + n_left) in edges
    # a lane change from right lane node 0 (x=0) lands about 10 m ahead in the left lane (x=10)
    lc = [b for a, b, _ in g.edges if a == 0 and n_right <= b < n_right + n_left]
    assert lc and g.nodes[lc[0]][0] == 10.0

    out = tmp_path / "g.json"
    g.to_json(str(out))
    data = json.loads(out.read_text())
    assert data["frame"] == "map" and len(data["nodes"]) == len(g.nodes)


def test_no_lane_change_against_traffic():
    a = straight_lane(-1, 0.0)
    b = straight_lane(1, 3.5)  # opposite direction
    a.left_key, a.can_change_left = b.key, True
    g = build_lane_graph([a, b])
    assert not [e for e in g.edges if e[0] < len(a.points) <= e[1]]
