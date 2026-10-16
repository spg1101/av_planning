# Architecture: nodes, topics and frames

## Data flow

```
platform (gym / kinematic sim / bag / CARLA / car)
   |  /sensors/scan  /sensors/imu  /vehicle/odom  /perception/detections
   v
localization: EKF (odom->base_link, /odometry/filtered) + SLAM (map->odom, /map)
tracker: /perception/detections -> /tracking/objects
   v
world_model: /world/ego_state  /world/costmap  /world/obstacles
   v
global_planner: /goal_pose -> /planning/route
behavior_planner: -> /planning/behavior
local_planner: -> /planning/trajectory  ==> controls (or pure_pursuit_stub) -> /drive
```

## Topics owned by planning

| Topic | Type | Publisher |
| --- | --- | --- |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox |
| `/tracking/objects` | `av_msgs/TrackedObjectArray` | tracker |
| `/world/ego_state` | `nav_msgs/Odometry` (map frame) | world_model |
| `/world/costmap` | `nav_msgs/OccupancyGrid` | world_model |
| `/world/obstacles` | `av_msgs/TrackedObjectArray` | world_model |
| `/planning/route` | `nav_msgs/Path` (latched) | global_planner |
| `/planning/behavior` | `av_msgs/BehaviorCommand` | behavior_planner |
| `/planning/trajectory` | `av_msgs/Trajectory` | local_planner |

## Who publishes each transform, per run mode

| Mode | map -> odom | odom -> base_link | base_link -> laser | Notes |
| --- | --- | --- | --- | --- |
| kinematic | -- | -- | -- | kinematic_sim publishes `map -> base_link` directly |
| gym | slam_toolbox | EKF | gym_adapter (static) | gym's own `ego_racecar/*` frames are separate ground truth |
| bag replay | slam_toolbox | EKF | from `/tf_static` in the bag | |
| CARLA / car | slam_toolbox | EKF | adapter / URDF | |

Only one node may publish each transform.
