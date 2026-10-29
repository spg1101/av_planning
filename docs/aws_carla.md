# Working on CARLA in AWS

## The setup in one picture

```
your laptop                                   AWS GPU instance (Ubuntu 22.04)
──────────                                    ───────────────────────────────
VS Code  ── Remote-SSH ─────────────────────▶ ~/planning_ws (this repo, built)
Foxglove ── ssh -L 8765:localhost:8765 ─────▶ foxglove_bridge (viz:=true)
                                              CARLA server  :2000 (Docker, headless)
                                              planning stack (devcontainer or native)
```

Run the stack **on the same instance as CARLA**. Synchronous mode makes the adapter wait for
every frame; doing that across the internet adds latency to every tick.

## Questions for the leads (answers go here)

| Question | Answer |
| --- | --- |
| CARLA version on the instance (client must match exactly) | 0.9.16 assumed |
| Instance type (needs an NVIDIA GPU with 8 GB+ VRAM; e.g. g5 or g4dn families) | |
| One instance per person, or shared? | |
| How do we start/stop it, and who pays attention to idle time? | |
| Is CARLA installed natively or via Docker? | |
| Ports open, or SSH only? (SSH only is fine and safer) | |

If the version is not 0.9.16, change `CARLA_VERSION` in `.devcontainer/Dockerfile` and in
`scripts/start_carla.sh` (or set the `CARLA_VERSION` environment variable).

## Sharing CARLA between people

Only **one** client may tick a server in synchronous mode. Two people on one server will break
each other's runs. Options, best first:

1. One instance each.
2. One big instance, one server per person on different ports:
   `PORT=2000 scripts/start_carla.sh` and `PORT=3000 scripts/start_carla.sh`, then
   `carla.launch.py port:=3000`. Each server uses `PORT`, `PORT+1` and `PORT+2`, and the
   Traffic Manager needs its own port too (set the adapter's `tm_port`, e.g. 8000 and 8100).
   Also give each person a different `ROS_DOMAIN_ID` so their ROS topics don't mix.

## Daily workflow

```bash
# on your laptop
ssh -L 8765:localhost:8765 ubuntu@<instance-ip>

# on the instance
cd ~/planning_ws/src/av_planning
scripts/start_carla.sh                                  # once per instance boot
cd ~/planning_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch planning_bringup carla.launch.py monitor:=false viz:=true
```

Then open Foxglove on your laptop, connect to `ws://localhost:8765`, and add panels for
`/sensors/lidar/points`, `/world/costmap`, `/planning/route` and `/planning/trajectory`.

Run the whole regression suite before opening a PR that touches planning:

```bash
scripts/run_scenarios.sh            # prints PASS/FAIL per scenario, results in ~/av_results
```

## First time on a new town

```bash
ros2 run planning_carla list_spawn_points --town Town03 > ~/Town03_spawn_points.csv
ros2 run planning_carla export_lane_graph --town Town03 \
    --out ~/planning_ws/src/av_planning/planning_bringup/maps/Town03_lane_graph.json
ros2 run planning_carla export_vehicle_params --blueprint vehicle.lincoln.mkz_2020 \
    --out ~/planning_ws/src/av_planning/planning_bringup/config/vehicles/carla.yaml
colcon build --symlink-install --packages-select planning_bringup
```

Commit the lane graph and the vehicle file so nobody else needs to export them again.

## Visualizing

- **Foxglove** (recommended): runs in a browser or desktop app on your laptop, only needs the
  SSH tunnel above. `viz:=true` starts the bridge.
- **The CARLA window**: CARLA runs headless (`-RenderOffScreen`). To watch it, use a remote
  desktop such as NICE DCV on the instance and start CARLA without `-RenderOffScreen`; the
  adapter keeps the spectator camera behind the ego car.
- **RViz**: works through the same remote desktop.

## Cost and hygiene

- Stop the instance when you finish for the day, even if the leads pay for it.
- `docker rm -f carla-2000` stops the server; the adapter restores CARLA to asynchronous mode
  when it exits, so a crashed run doesn't leave the server frozen. If it does freeze, restart
  the server with `scripts/start_carla.sh`.
- Bags are large (lidar): record only what you need and copy them off the instance.
