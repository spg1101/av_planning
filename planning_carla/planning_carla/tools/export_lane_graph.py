"""Export a CARLA town's road network as a lane graph JSON for the global planner.

Run once per town (CARLA server must be running; no ROS needed):

    ros2 run planning_carla export_lane_graph --town Town03 \\
        --out src/av_planning/planning_bringup/maps/Town03_lane_graph.json

Every lane is sampled every --spacing metres. Edges connect consecutive points, the end of
each lane to the lanes that continue from it (including through junctions), and, where road
markings allow, to the neighbouring lane in the same direction (lane changes, with a cost
penalty so the route only changes lanes when it needs to).
"""

import argparse

import carla

from planning_carla.lane_graph_builder import LaneSegment, build_lane_graph
from planning_carla.transforms import location_to_ros, rotation_to_ros


def key(wp):
    return (wp.road_id, wp.section_id, wp.lane_id)


def sample_segment(start, end, spacing):
    """Waypoints from start to end along one lane, about `spacing` metres apart."""
    wps = [start]
    wp = start
    limit = 10000
    while limit > 0 and wp.transform.location.distance(end.transform.location) > spacing:
        nxt = [n for n in wp.next(spacing) if key(n) == key(start)]
        if not nxt:
            break
        wp = nxt[0]
        wps.append(wp)
        limit -= 1
    if wps[-1].transform.location.distance(end.transform.location) > 0.1:
        wps.append(end)
    return wps


def to_segment(wps):
    pts, s = [], []
    for wp in wps:
        t = wp.transform
        x, y, _ = location_to_ros(t.location.x, t.location.y, t.location.z)
        _, _, yaw = rotation_to_ros(0.0, 0.0, t.rotation.yaw)
        pts.append((x, y, yaw))
        s.append(wp.s)
    first = wps[0]
    left, right = first.get_left_lane(), first.get_right_lane()
    lc = first.lane_change
    return LaneSegment(
        key=key(first),
        points=pts,
        s=s,
        left_key=key(left) if left is not None and left.lane_type == carla.LaneType.Driving else None,
        right_key=key(right) if right is not None and right.lane_type == carla.LaneType.Driving else None,
        can_change_left=lc in (carla.LaneChange.Left, carla.LaneChange.Both),
        can_change_right=lc in (carla.LaneChange.Right, carla.LaneChange.Both),
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--town", default="", help="load this town first (e.g. Town03)")
    ap.add_argument("--spacing", type=float, default=2.0)
    ap.add_argument("--lane-change-distance", type=float, default=12.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    if args.town and not world.get_map().name.endswith(args.town):
        world = client.load_world(args.town)
    cmap = world.get_map()

    segments = [to_segment(sample_segment(a, b, args.spacing)) for a, b in cmap.get_topology()]
    graph = build_lane_graph(segments, lane_change_distance=args.lane_change_distance)
    graph.to_json(args.out)
    print(
        f"{cmap.name}: {len(segments)} lane segments, {len(graph.nodes)} nodes, "
        f"{len(graph.edges)} edges -> {args.out}"
    )


if __name__ == "__main__":
    main()
