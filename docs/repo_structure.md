# How this repo is structured, and why

This document explains every folder in `av_planning`, the reasoning behind the layout,
and how the pieces connect at runtime. Read it once before your first PR.

## 1. The big picture: three repos, one workspace

| Repo | Owner | What it holds |
| --- | --- | --- |
| `av_planning` (this repo) | Planning subteam | All planning code, configs, launch files, tests |
| `av_interfaces` | Planning lead + controls lead | The shared message types (`av_msgs`) and the interface contract (`ICD.md`) |
| `f1tenth_gym_ros` | External (F1TENTH project) | The simulator bridge we develop against |

None of these is copied into another. `planning.repos` lists the other two, and
`vcs import < planning.repos` clones them next to this repo inside one colcon workspace:

```
~/planning_ws/src/
  av_planning/        <- this repo
  av_interfaces/      <- pulled in
  f1tenth_gym_ros/    <- pulled in
```

`colcon build` treats every folder with a `package.xml` as a package, no matter which repo
it came from. That is the whole integration story: later, the team adds a `team.repos`
listing every subteam's repo, and the combined stack builds the same way.

**Why messages live in their own repo.** If planning and controls each kept a copy of
`Trajectory.msg`, the copies would drift, and integration day would start with a
message mismatch. With one shared repo there is exactly one definition, and changing it
requires both leads to approve.

## 2. The three layers inside av_planning

The packages fall into three layers. Dependencies only point downward.

| Layer | Packages | Knows about ROS? | Changes when... |
| --- | --- | --- | --- |
| Algorithms | `planning_core` | No | the math changes |
| ROS wrappers | `planning_nodes`, `planning_world_model`, `planning_localization`, `planning_sim`, `planning_eval` | Yes | topics, parameters or timing change |
| Composition | `planning_bringup` | Only launch | you switch platform or run mode |

**Why separate the algorithms from ROS.** An A* search, a Frenet transform or a
velocity profile doesn't need ROS. Keeping them in plain Python means:

- tests run in seconds with `pytest`, no simulator, no ROS graph;
- the same code runs unchanged on the kinematic sim, the gym, bag replay, CARLA and the car;
- a reviewer can read the algorithm without wading through subscriber boilerplate.

The rule is enforced by convention and review: **nothing in `planning_core` imports `rclpy`,
ROS messages or any simulator.** The single file that translates between ROS messages and
core types is `planning_nodes/planning_nodes/conversions.py`.

## 3. Package by package

Every top-level `planning_*` folder is one ROS 2 package, built with `ament_python`. Each has
the same skeleton:

```
planning_x/
  package.xml          # name, dependencies (rosdep reads this)
  setup.py             # Python packaging + console_scripts = the node executables
  setup.cfg            # tells colcon where to install executables
  resource/planning_x  # empty marker file so ROS can find the package
  planning_x/          # the Python module itself
  test/                # pytest files, run by `colcon test`
```

### planning_core: the algorithms (no ROS)

| Module | Contents | Owner |
| --- | --- | --- |
| `types.py` | `EgoState`, `TrajectoryPoint`, `Trajectory`, `Obstacle` dataclasses | Lead |
| `vehicle.py` | `VehicleParams` loaded from the shared vehicle YAML | Lead |
| `geometry/path.py` | arc length, resampling, headings, curvature of a polyline | Member B |
| `geometry/frenet.py` | `ReferencePath`: Cartesian <-> Frenet (s, d) | Member B |
| `global_planner/astar.py` | generic A* over a weighted graph | Member B |
| `global_planner/lane_graph.py` | waypoint loop / lane graph, nearest node | Member B |
| `behavior/state_machine.py` | FOLLOW_LANE / STOP / YIELD / OVERTAKE / E_STOP with hysteresis | Lead |
| `local_planner/lattice.py` | quintic lateral candidates in the Frenet frame | Member B |
| `local_planner/collision.py` | checks against the costmap and predicted obstacles | Member B |
| `local_planner/cost.py` | candidate scoring | Member B |
| `trajectory/velocity_profile.py` | curvature- and acceleration-limited speed profile | Member B |
| `trajectory/stitching.py` | start each replan from the previous plan | Lead |

The `test/` folder already has working tests for the geometry, velocity profile, A*,
behavior and stitching code. New algorithm code comes with tests in the same PR.

### planning_nodes: the planners as ROS nodes

| Executable | Subscribes | Publishes |
| --- | --- | --- |
| `global_planner` | `/goal_pose`, `/world/ego_state` | `/planning/route` (latched `nav_msgs/Path`) |
| `behavior_planner` | `/world/ego_state`, `/world/obstacles` | `/planning/behavior` (`av_msgs/BehaviorCommand`) |
| `local_planner` | route, behavior, ego state, costmap, obstacles | `/planning/trajectory` (`av_msgs/Trajectory`) |

Each node is thin: read messages, convert with `conversions.py`, call `planning_core`,
convert back, publish. The local planner ships as version 0, which follows the route
centerline with a proper velocity profile and honours STOP and E_STOP. That makes the
full loop runnable on day one; Member B replaces the inside with the lattice planner
without touching any other package.

### planning_world_model: what the planner sees

| Executable | Subscribes | Publishes |
| --- | --- | --- |
| `world_model` | TF `map -> base_link`, velocity odometry, `/map`, `/tracking/objects` | `/world/ego_state`, `/world/costmap`, `/world/obstacles` |
| `tracker` | `/perception/detections` (`vision_msgs/Detection3DArray`) | `/tracking/objects` |

The world model is the single place planners read from, so the planners never have to
know whether the pose came from SLAM, the kinematic sim or CARLA. It inflates the map by
the car's radius (so planners can treat the car as a point) and adds constant-velocity
predictions to each tracked object. The tracker starts as nearest-neighbour association;
the plan upgrades it to a Kalman filter with Hungarian matching.

### planning_localization: EKF and SLAM

No Python logic here, only configuration and one launch file, because both algorithms come
from mature packages: `robot_localization` (EKF) and `slam_toolbox` (SLAM).

| File | Purpose |
| --- | --- |
| `config/ekf_gym.yaml` | wheel odometry only (the gym has no IMU) |
| `config/ekf_f1tenth.yaml` | wheel odometry + IMU |
| `config/ekf_carla.yaml` | wheel odometry + IMU, GNSS to be added |
| `config/slam_mapping.yaml` | build a map while driving |
| `config/slam_localization.yaml` | localize against a saved map |
| `launch/localization.launch.py` | `platform:=gym|f1tenth|carla`, `slam_mode:=mapping|localization|off` |

The two estimators split the job:

- **EKF** publishes `odom -> base_link`: smooth and high-rate, but slowly drifting.
- **SLAM** publishes `map -> odom`: the correction that removes the drift.

Together any node can look up `map -> base_link` and get the best pose estimate.
`slam_mode:=off` replaces SLAM with an identity transform, which is handy for tuning the
EKF alone.

### planning_sim: stand-ins for the other subteams

This package exists so planning never waits on another subteam. Each stand-in uses the
same topics as the real component, so swapping in the real one is a launch-file edit.

| Executable | Stands in for | What it does |
| --- | --- | --- |
| `gym_adapter` | Integration team's platform adapter | Renames gym topics to ours, adds odometry noise, publishes `base_link -> laser` |
| `kinematic_sim` | A simulator | Bicycle-model car; `perfect_tracking:=true` snaps the car onto each trajectory |
| `pure_pursuit_stub` | Controls team | Pure pursuit steering + speed from the trajectory; stops on timeout or E_STOP |
| `fake_perception` | Perception team | Turns other cars' ground-truth odometry into noisy detections |
| `canned_trajectory` | (Planning, for controls) | Straight / circle / lane change / stop / speed ramp trajectories for controls tuning |
| `waypoint_recorder` | (Tool) | Drive a lap, save a waypoint CSV for the global planner |

At integration time this whole package is simply left out of the team launch files.

### planning_eval: measuring

| Item | Purpose |
| --- | --- |
| `metrics.py` | pure functions: time matching, RMSE (unit-tested) |
| `localization_error` | live node comparing `/world/ego_state` with `/ground_truth/odom`, prints RMSE |

Every localization PR quotes a number from this node, which turns "it looks better" into
"RMSE dropped from 0.31 m to 0.12 m on bag gym_levine_01".

### planning_bringup: putting it together

| Item | Purpose |
| --- | --- |
| `launch/kinematic.launch.py` | kinematic sim + full planning stack + pure pursuit; no gym needed |
| `launch/gym.launch.py` | full stack with EKF + SLAM on f1tenth_gym_ros |
| `launch/bag_replay.launch.py` | localization + world model re-run on a recorded bag, scored live |
| `launch/carla.launch.py` | placeholder until the CARLA adapter exists |
| `config/vehicles/*.yaml` | wheelbase, steering and acceleration limits per platform, shared with controls |
| `maps/` | waypoint CSVs and saved SLAM maps |

Configs and maps live inside this package (rather than at the repo root) so `colcon`
installs them and launch files can find them with `get_package_share_directory`.
Switching platforms means picking a different launch file and vehicle YAML; no node code
changes.

## 4. How a message travels through the stack

1. The platform (gym, kinematic sim, bag, CARLA, car) produces `/sensors/scan`,
   `/vehicle/odom` and, where available, `/sensors/imu` and `/perception/detections`.
2. The EKF fuses odometry and IMU into `odom -> base_link`; SLAM matches scans to the map
   and publishes `map -> odom` and `/map`.
3. The tracker turns detections into objects with stable IDs and velocities.
4. The world model publishes the ego state in the map frame, the inflated costmap and
   predicted obstacles.
5. The global planner publishes a route once; the behavior planner decides what to do
   (follow, stop, ...); the local planner turns both into a time-stamped trajectory.
6. The controller (or `pure_pursuit_stub`) converts the trajectory into `/drive`.

`docs/architecture.md` has the full topic table and which node publishes each transform
in each run mode.

## 5. Run modes, cheapest first

| Mode | Needs | Tests |
| --- | --- | --- |
| `colcon test` | nothing | algorithms |
| `kinematic.launch.py` | laptop | planning logic and the full topic plumbing |
| `bag_replay.launch.py` | a recorded bag | EKF, SLAM, world model against ground truth |
| `gym.launch.py` | f1tenth_gym_ros | everything closed loop with a simulated lidar |
| `carla.launch.py` | CARLA + adapter | full-scale validation (scheduled sessions only) |
| the car | hardware | final tests |

Most days should be spent in the first three rows.

## 6. Tooling and review

| File | Purpose |
| --- | --- |
| `.devcontainer/` | Docker image with ROS 2 Humble and every dependency; identical setup for all three of us |
| `.pre-commit-config.yaml`, `pyproject.toml` | `ruff` lint and format, `mypy` on `planning_core`, run on every commit |
| `.github/workflows/ci.yml` | on every PR: lint, then build and test in a `ros:humble` container |
| `.github/CODEOWNERS` | the planning lead must approve every PR; members co-own the lead's areas so the lead's own PRs can be approved |
| `.github/pull_request_template.md` | task ID, "done when" evidence, metrics |
| `CONTRIBUTING.md` | branch naming, review rules, GitHub branch protection settings |
| `planning.repos` / `ci.repos` | external repos for local work / for CI (CI skips the simulator) |

## 7. How integration with the other subteams will work

1. Every subteam keeps its own repo with prefixed package names (`planning_*`, `control_*`, ...).
2. Someone adds a `team.repos` listing all subteam repos plus `av_interfaces`.
3. A team bringup package launches the real adapters, perception and controls with our
   `planning_localization`, `planning_world_model` and `planning_nodes`, and leaves
   `planning_sim` out.

Because topics, frames and messages already follow the ICD, step 3 is mostly launch-file work.

## 8. Quick answers

- **Where do I put a new algorithm?** In `planning_core`, with a test. Then call it from a node.
- **Where do I put a new parameter?** `declare_parameter` in the node, value in the launch file or a YAML.
- **I need a new message field.** Open a PR on `av_interfaces`; both leads approve.
- **Why Python and not C++?** Faster to write and review for a team of three. If profiling
  shows a hot loop, move just that function to C++ later.
- **Why Humble?** It's a long-term-support release on Ubuntu 22.04 with packages for
  everything we use (`robot_localization`, `slam_toolbox`, `ackermann_msgs`). Note that
  f1tenth_gym_ros's own README documents Foxy on Ubuntu 20.04, so expect small fixes the
  first time it builds on Humble; pin the commit that works in `planning.repos`.
