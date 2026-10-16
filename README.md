# av_planning

Planning subteam repo (ROS 2 **Humble**, Ubuntu 22.04): localization (EKF + SLAM),
world model, global / behavior / local planners, and stand-ins for the other subteams
so the whole loop runs on a laptop.

Read [`docs/repo_structure.md`](docs/repo_structure.md) for what every folder is for.

## Setup (once)

```bash
mkdir -p ~/planning_ws/src && cd ~/planning_ws/src
git clone git@github.com:<your-org>/av_planning.git
vcs import < av_planning/planning.repos          # pulls av_interfaces + f1tenth_gym_ros
cd ~/planning_ws
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
cd src/av_planning && pre-commit install         # lint on every commit
```

Or open the repo in VS Code and choose **Reopen in Container**: `.devcontainer/` has
ROS 2 Humble and every dependency preinstalled.

## Run it

| Command | What runs | Use it for |
| --- | --- | --- |
| `ros2 launch planning_bringup kinematic.launch.py` | Kinematic sim + planners + pure pursuit stub | Fastest planning loop, no gym |
| `... kinematic.launch.py perfect_tracking:=true` | Car snaps onto each trajectory | Planning logic without any controller |
| `ros2 launch planning_bringup gym.launch.py` | Full stack incl. EKF + SLAM on f1tenth_gym | Closed loop with lidar |
| `ros2 launch planning_bringup bag_replay.launch.py bag:=<path>` | EKF + SLAM + world model on a bag | Localization work, scored vs ground truth |
| `colcon test && colcon test-result --verbose` | All unit tests | Before every PR |

For `gym.launch.py`, start the simulator first in another terminal:
`ros2 launch f1tenth_gym_ros gym_bridge_launch.py`.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md). Every PR is reviewed by the planning lead.
