# Architecture: CARLA-first

CARLA on AWS is the main development platform. Every node talks only to standard topics, so
the same stack later runs on the F1TENTH car by swapping the platform layer.

## Layers

```
┌ platform ─────────────────────────────────────────────────────────────────────┐
│ CARLA server (AWS GPU)  <->  carla_adapter (only node that imports `carla`)    │
│   also: kinematic_sim (laptop/CI), bag replay, later the F1TENTH car driver    │
└──── /sensors/*  /vehicle/odom  /clock  /ground_truth/*  /goal_pose  ▲ /drive ──┘
        │                                                             │
┌ localization ─────────────┐  ┌ perception (other team) ┐            │
│ EKF: odom -> base_link    │  │ /perception/detections  │            │
│ SLAM or GT: map -> odom   │  │ (fake_perception today) │            │
└────────────┬──────────────┘  └───────────┬─────────────┘            │
             │                    tracker -> /tracking/objects         │
┌ world model ┴────────────────────────────┴──────────────┐           │
│ /world/ego_state  /world/costmap  /world/obstacles       │           │
└───────────────────────────┬──────────────────────────────┘           │
┌ planning ─────────────────┴──────────────────────────────┐          │
│ global_planner (lane graph + A*) -> /planning/route        │          │
│ behavior_planner (follow / stop / ...) -> /planning/behavior          │
│ local_planner -> /planning/trajectory ─────────────────────┼──> controls (other team,
└────────────────────────────────────────────────────────────┘     or pure_pursuit_stub) ┘
```

## Topics

| Topic | Type | Publisher | Notes |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | carla_adapter | simulation time; every other node runs `use_sim_time` |
| `/sensors/lidar/points` | `sensor_msgs/PointCloud2` | carla_adapter | 32-beam lidar, frame `lidar` |
| `/sensors/scan` | `sensor_msgs/LaserScan` | pointcloud_to_laserscan | 0.5-2.0 m slice of the lidar, for SLAM + costmap |
| `/sensors/imu` | `sensor_msgs/Imu` | carla_adapter | frame `imu` |
| `/sensors/gnss` | `sensor_msgs/NavSatFix` | carla_adapter | frame `gnss` |
| `/sensors/camera/image` | `sensor_msgs/Image` | carla_adapter | off by default (perception team) |
| `/vehicle/odom` | `nav_msgs/Odometry` | carla_adapter | twist only, with noise: "wheel odometry" |
| `/ground_truth/odom` | `nav_msgs/Odometry` | carla_adapter | map frame; evaluation and `ground_truth` mode only |
| `/ground_truth/objects` | `av_msgs/TrackedObjectArray` | carla_adapter | all vehicles/walkers within 80 m |
| `/ground_truth/collision` | `geometry_msgs/Vector3Stamped` | carla_adapter | frame_id = what we hit |
| `/goal_pose` | `geometry_msgs/PoseStamped` (latched) | carla_adapter | from the scenario's `goal_index` |
| `/perception/detections` | `vision_msgs/Detection3DArray` | perception team / fake_perception | map frame |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (or ground_truth_localization) | odom frame |
| `/tracking/objects` | `av_msgs/TrackedObjectArray` | tracker | stable IDs + velocities |
| `/world/ego_state` | `nav_msgs/Odometry` | world_model | map frame, 50 Hz |
| `/world/costmap` | `nav_msgs/OccupancyGrid` | world_model | 60 m rolling window from the scan, inflated |
| `/world/obstacles` | `av_msgs/TrackedObjectArray` | world_model | tracks + 2 s constant-velocity predictions |
| `/planning/route` | `nav_msgs/Path` (latched) | global_planner | lane centrelines start -> goal |
| `/planning/behavior` | `av_msgs/BehaviorCommand` | behavior_planner | state, speed limit, stop point |
| `/planning/trajectory` | `av_msgs/Trajectory` | local_planner | the ICD output to controls |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | controls / pure_pursuit_stub | speed + steering angle |

## Frames

- `map` = CARLA's world frame with **y flipped** (ROS is right-handed, CARLA left-handed). The
  lane graph, goal and ground truth are all in this frame.
- `base_link` = **rear axle centre** (the ICD reference point), x forward, y left.
- In CARLA the adapter sends the true start pose to the EKF before the first tick, so `odom`
  and the SLAM `map` start aligned with CARLA's world frame.

| Localization mode | map -> odom | odom -> base_link | Use it for |
| --- | --- | --- | --- |
| `ground_truth` (default) | identity | ground truth | planning work; isolates planning from localization bugs |
| `ekf_only` | identity | EKF | tuning the EKF alone (drift is expected) |
| `slam_mapping` | slam_toolbox | EKF | the real stack, building a map |
| `slam_localization` | slam_toolbox (saved map) | EKF | the real stack on a known town |
| kinematic sim | -- | -- | kinematic_sim publishes `map -> base_link` itself |

Only one node may publish each transform.

## Timing

CARLA runs in **synchronous mode** at 20 Hz (`fixed_delta_seconds: 0.05`). The adapter is
the only client that ticks; after each tick it waits for that frame's sensor data and publishes
everything with the simulation timestamp. With `realtime_factor: 1.0` one simulated second
takes one real second; lower it if the stack can't keep up, and results stay identical.

## Test ladder

| Level | Command | Needs | Catches |
| --- | --- | --- | --- |
| Unit tests | `colcon test` | laptop / CI | algorithm bugs |
| Kinematic loop | `ros2 launch planning_bringup kinematic.launch.py` | laptop / CI | planning logic, topic plumbing |
| CARLA, interactive | `carla.launch.py monitor:=false viz:=true` | AWS | everything, watched live |
| CARLA, scenario suite | `scripts/run_scenarios.sh` | AWS | regressions: pass/fail per scenario |
| Bag replay | `bag_replay.launch.py bag:=...` | laptop | localization tuning on identical data |
| F1TENTH car | (Phase 4: f1tenth adapter) | hardware | the real world |
