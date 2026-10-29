"""Print a town's spawn points (index, x, y, yaw in the ROS map frame) to pick scenario
start and goal indices.

    ros2 run planning_carla list_spawn_points --town Town03 > Town03_spawn_points.csv
"""

import argparse

import carla

from planning_carla.transforms import location_to_ros, rotation_to_ros


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--town", default="")
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    if args.town and not world.get_map().name.endswith(args.town):
        world = client.load_world(args.town)
    print("# index, x, y, yaw  (m, m, rad; ROS map frame)  " + world.get_map().name)
    for i, sp in enumerate(world.get_map().get_spawn_points()):
        x, y, _ = location_to_ros(sp.location.x, sp.location.y, sp.location.z)
        _, _, yaw = rotation_to_ros(0.0, 0.0, sp.rotation.yaw)
        print(f"{i}, {x:.1f}, {y:.1f}, {yaw:.2f}")


if __name__ == "__main__":
    main()
