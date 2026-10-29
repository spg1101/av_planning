# av_planning

Planning subteam repo (ROS 2 **Humble**, Ubuntu 22.04), developed on **CARLA** running on AWS:
localization (EKF + SLAM), world model, global / behavior / local planners, a CARLA adapter,
and stand-ins for the other subteams so the whole loop runs today.

- [`docs/repo_structure.md`](docs/repo_structure.md): what every folder is for
- [`docs/architecture.md`](docs/architecture.md): nodes, topics, frames, test ladder
- [`docs/aws_carla.md`](docs/aws_carla.md): working on the AWS instance

## Setup (once, on the AWS instance)

```bash
mkdir -p ~/planning_ws/src && cd ~/planning_ws/src
git clone git@github.com:<your-org>/av_planning.git
vcs import < av_planning/planning.repos            # pulls av_interfaces
python3 -m pip install carla==0.9.16               # must match the CARLA server version
cd ~/planning_ws
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
cd src/av_planning && pre-commit install           # lint on every commit
```

Or open the repo in VS Code (Remote-SSH to the instance) and choose **Reopen in Container**:
`.devcontainer/` has ROS 2 Humble, the CARLA client and every dependency preinstalled.

Before the first CARLA run, export the town's lane graph (see `docs/aws_carla.md`).

## Run it

| Command | What runs | Use it for |
| --- | --- | --- |
| `scripts/start_carla.sh` | Headless CARLA server (Docker) | Once per instance boot |
| `ros2 launch planning_bringup carla.launch.py monitor:=false viz:=true` | Full stack in CARLA, Foxglove on :8765 | Day-to-day work |
| `ros2 launch planning_bringup carla.launch.py scenario:=town03_route_traffic` | One scenario, judged PASS/FAIL | Checking a change |
| `scripts/run_scenarios.sh` | Every scenario, summary table | Before every planning PR |
| `... carla.launch.py localization:=slam_mapping record:=t03_lap1` | Real localization, recorded to a bag | Collecting data |
| `ros2 launch planning_bringup bag_replay.launch.py bag:=t03_lap1` | EKF + SLAM on a bag, scored vs ground truth | Localization tuning, no CARLA |
| `ros2 launch planning_bringup kinematic.launch.py` | Kinematic sim + planners, no CARLA | Quick logic checks on a laptop |
| `colcon test && colcon test-result --verbose` | All unit tests | Before every PR |

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md). Every PR is reviewed by the planning lead.
