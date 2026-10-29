"""Write the shared vehicle YAML from CARLA's own physics for a vehicle blueprint.

    ros2 run planning_carla export_vehicle_params --blueprint vehicle.lincoln.mkz_2020 \\
        --out src/av_planning/planning_bringup/config/vehicles/carla.yaml

Wheelbase, steering limit and size come from CARLA. The acceleration and comfort limits are
team choices and are kept from the existing file if one is there.
"""

import argparse
import math
import os

import carla
import numpy as np
import yaml

DEFAULT_LIMITS = {
    "max_speed": 13.9,
    "max_accel": 3.0,
    "max_decel": 6.0,
    "max_lateral_accel": 3.0,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--blueprint", default="vehicle.lincoln.mkz_2020")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    bp = world.get_blueprint_library().find(args.blueprint)
    spawn = world.get_map().get_spawn_points()[0]
    vehicle = world.try_spawn_actor(bp, spawn)
    if vehicle is None:
        raise SystemExit("spawn point 0 is occupied; clear the world and retry")
    try:
        world.tick() if world.get_settings().synchronous_mode else world.wait_for_tick()
        physics = vehicle.get_physics_control()
        inv = np.array(vehicle.get_transform().get_inverse_matrix())
        xs = [
            (inv @ np.array([w.position.x / 100, w.position.y / 100, w.position.z / 100, 1.0]))[0]
            for w in physics.wheels
        ]
        wheelbase = 0.5 * (xs[0] + xs[1]) - 0.5 * (xs[2] + xs[3])
        extent = vehicle.bounding_box.extent
    finally:
        vehicle.destroy()

    limits = dict(DEFAULT_LIMITS)
    if os.path.exists(args.out):
        with open(args.out) as f:
            old = (yaml.safe_load(f) or {}).get("vehicle", {})
        limits.update({k: old[k] for k in DEFAULT_LIMITS if k in old})

    data = {
        "vehicle": {
            "wheelbase": round(float(wheelbase), 4),
            "max_steer": round(math.radians(physics.wheels[0].max_steer_angle), 4),
            **limits,
            "width": round(2 * extent.y, 3),
            "length": round(2 * extent.x, 3),
        }
    }
    with open(args.out, "w") as f:
        f.write(
            f"# CARLA {args.blueprint}, exported by export_vehicle_params. Shared by planning AND controls.\n"
        )
        yaml.safe_dump(data, f, sort_keys=False)
    print(yaml.safe_dump(data, sort_keys=False))


if __name__ == "__main__":
    main()
