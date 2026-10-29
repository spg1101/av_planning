# How this repo is structured, and why

## 1. The big picture

We develop on **CARLA, running on an AWS GPU instance**, and later move the same stack to the
F1TENTH car. Everything is organized so that only one small layer knows which platform it is on.

| Repo | Owner | What it holds |
| --- | --- | --- |
| `av_planning` (this repo) | Planning subteam | All planning code, the CARLA adapter, configs, launch files, tests |
| `av_interfaces` | Planning lead + controls lead | The shared messages (`av_msgs`) and the interface contract (`ICD.md`) |

`planning.repos` pulls `av_interfaces` into the same colcon workspace. Integration with the
other subteams later works the same way: a `team.repos` lists every subteam's repo.

## 2. Four layers

| Layer | Packages | Knows about | Changes when... |
| --- | --- | --- | --- |
| Algorithms | `planning_core` | numpy only | the math changes |
| Platform | `planning_carla`, `planning_sim` | CARLA / simulators | we change simulator or move to the car |
| ROS nodes | `planning_nodes`, `planning_world_model`, `planning_localization`, `planning_eval` | ROS topics | topics, parameters or timing change |
| Composition | `planning_bringup` | launch files, configs, scenarios | we change what runs together |

Two rules make this work:

- **Nothing in `planning_core` imports ROS or CARLA.** Algorithms are unit-tested in seconds
  and run unchanged on every platform.
- **Only `planning_carla/carla_adapter.py` imports `carla`.** Every other node sees standard
  topics, so moving to the F1TENTH car means writing one new adapter, not touching planners.

## 3. Package by package

### planning_core: algorithms (no ROS)

| Module | Contents | Owner |
| --- | --- | --- |
| `types.py`, `vehicle.py` | data types; vehicle limits from the shared YAML | Lead |
| `geometry/` | polylines, curvature, Frenet (s, d) conversions | Member B |
| `global_planner/` | A*, lane graph (CARLA JSON or waypoint CSV), heading-aware nearest node | Member B |
| `behavior/` | state machine, lead-vehicle following | Lead |
| `local_planner/` | lattice candidates, collision checks, cost | Member B |
| `trajectory/` | velocity profile, stitching | Member B / Lead |
| `world/costmap.py` | scan -> rolling grid, inflation | Member A |

### planning_carla: the CARLA platform layer (Lead)

| Item | What it does |
| --- | --- |
| `carla_adapter` | Connects to CARLA, runs it in synchronous mode (the only client that ticks), spawns the ego car, its sensors, traffic and parked cars, converts `/drive` to throttle/brake/steer, and publishes sensors, odometry, ground truth, `/clock` and the goal |
| `transforms.py`, `control.py`, `lane_graph_builder.py` | The pure math behind it (CARLA's left-handed frame to ROS, speed PID, graph building), all unit-tested without CARLA |
| `export_lane_graph` | One-time tool: the town's OpenDRIVE road network -> `maps/<Town>_lane_graph.json` |
| `export_vehicle_params` | One-time tool: wheelbase and steering limit from CARLA physics -> `vehicles/carla.yaml` |
| `list_spawn_points` | Prints spawn point indices to pick scenario starts and goals |
| `config/sensors.yaml` | The sensor suite: 32-beam lidar, IMU, GNSS, optional camera, collision sensor |

Why our own adapter instead of the official CARLA ROS bridge: the official bridge supports
ROS 2 Foxy, not Humble (Humble needs a community fork). CARLA 0.9.16 can also publish sensors
to ROS 2 natively, but our adapter gives us exactly the topics in the ICD, ground truth for
evaluation, rear-axle frames, and full control of ticking and scenarios in one place.

### planning_localization: where am I? (Member A)

| Item | Purpose |
| --- | --- |
| `launch/localization.launch.py` | `mode:=ground_truth | ekf_only | slam_mapping | slam_localization` |
| `config/ekf_carla.yaml`, `ekf_f1tenth.yaml` | `robot_localization` EKF: wheel odometry + IMU |
| `config/slam_carla.yaml`, `slam_f1tenth.yaml` | `slam_toolbox` tuned per scale |
| `ground_truth_localization` | Perfect localization from CARLA, published exactly like the real one |
| `initial_pose_from_ground_truth` | Aligns replayed localization with ground truth in bag replay |

`ground_truth` is the default so planning work never waits on localization. Switching to the
real thing is one launch argument, which also makes it easy to see whether a failure comes
from localization or from planning.

### planning_world_model: what's around me? (Member A)

| Executable | In | Out |
| --- | --- | --- |
| `tracker` | `/perception/detections` | `/tracking/objects` (IDs + smoothed velocities) |
| `world_model` | TF, `/odometry/filtered`, `/sensors/scan`, `/tracking/objects` | `/world/ego_state`, `/world/costmap`, `/world/obstacles` |

Planners read only `/world/*`, so they don't care where the pose or obstacles came from.

### planning_nodes: the planners (Member B, behavior: Lead)

| Executable | In | Out |
| --- | --- | --- |
| `global_planner` | `/goal_pose`, `/world/ego_state`, lane graph file | `/planning/route` |
| `behavior_planner` | `/world/ego_state`, `/world/obstacles` | `/planning/behavior` (state, speed limit, stop point) |
| `local_planner` | route, behavior, ego state, costmap, obstacles | `/planning/trajectory` |

The local planner is a working version 0 (route following, speed profile, lead-vehicle
following, stop points, stop at the goal). Member B replaces its inside with the lattice planner.

### planning_sim: stand-ins (Lead)

| Executable | Stands in for |
| --- | --- |
| `fake_perception` | the perception team: CARLA ground-truth objects + noise, range limit, missed detections |
| `pure_pursuit_stub` | the controls team: trajectory -> `/drive` |
| `canned_trajectory` | test trajectories for the controls team |
| `kinematic_sim` | a simulator for quick checks without CARLA (and CI) |
| `waypoint_recorder` | recording a track for the F1TENTH phase |

Each uses the real topic names; `perception:=model` or `controller:=external` swaps in the real one.

### planning_eval: measuring

| Item | Purpose |
| --- | --- |
| `scenario_monitor` | Judges a CARLA run: PASS if the car stops at the goal in time without a collision; writes JSON, then ends the launch |
| `localization_error` | Live RMSE of the estimated pose vs ground truth |
| `metrics.py` | Pure metric functions (unit-tested) |

### planning_bringup: putting it together

| Item | Purpose |
| --- | --- |
| `launch/carla.launch.py` | The whole stack on CARLA; arguments pick scenario, localization mode, perception, controller, viz, recording |
| `launch/bag_replay.launch.py` | Localization on a recorded CARLA bag, no CARLA needed |
| `launch/kinematic.launch.py` | Planners on the kinematic sim, no CARLA needed |
| `config/scenarios/*.yaml` | Town, start, goal, traffic, parked cars, seed, timeout: one file per test case |
| `config/vehicles/*.yaml` | Vehicle limits shared with controls |
| `maps/` | Lane graphs per town, the example track, saved SLAM maps |

## 4. Scenarios are the team's test suite

A scenario file is a reproducible test: same town, start, goal, traffic and random seed every
time. `scripts/run_scenarios.sh` runs them all and prints PASS/FAIL. Every planning PR quotes the
suite result, and new behaviors arrive with a new scenario that proves them (for example, the
parked-car scenario fails today and passes once the lattice planner and overtaking are done).

## 5. Tooling and review

| File | Purpose |
| --- | --- |
| `.devcontainer/` | ROS 2 Humble + CARLA client + all dependencies; same environment for everyone |
| `scripts/start_carla.sh` | Headless CARLA server in Docker (or native), one per port |
| `scripts/run_scenarios.sh` | The scenario suite |
| `.pre-commit-config.yaml`, `pyproject.toml` | `ruff` and `mypy` on every commit |
| `.github/workflows/ci.yml` | Lint, build and unit tests on every PR (no CARLA in CI) |
| `.github/CODEOWNERS` | The planning lead approves every PR; members co-own the lead's areas |
| `CONTRIBUTING.md` | Branches, review rules, GitHub settings |

## 6. Moving to the F1TENTH car later

1. Write `planning_f1tenth` with one adapter node: VESC odometry, Hokuyo scan, IMU in;
   `/drive` out through `ackermann_mux`. Same topics as the CARLA adapter.
2. Use `vehicles/f1tenth.yaml`, `ekf_f1tenth.yaml`, `slam_f1tenth.yaml` (already here).
3. Route from a recorded waypoint CSV instead of a CARLA lane graph.

Planners, world model and localization code stay as they are.

## 7. Quick answers

- **New algorithm?** `planning_core` with a test, then call it from a node.
- **New test case?** A new file in `config/scenarios/`.
- **New message field?** A PR on `av_interfaces`, approved by both leads.
- **Something only CARLA can tell us** (traffic light state, lane IDs)? Add it to the adapter
  as a `/ground_truth/...` topic, never import `carla` anywhere else.
