"""The full planning stack on CARLA. Start the CARLA server first (scripts/start_carla.sh).

  ros2 launch planning_bringup carla.launch.py                                  # default scenario
  ros2 launch planning_bringup carla.launch.py scenario:=town03_route_traffic
  ros2 launch planning_bringup carla.launch.py localization:=slam_mapping record:=t03_lap1

Arguments (defaults first):
  scenario      town03_route_empty | <name in config/scenarios> | /abs/path/to/file.yaml
  localization  ground_truth | ekf_only | slam_mapping | slam_localization (+ map_file)
  perception    fake (ground truth + noise) | model (perception team publishes detections) | none
  controller    stub (pure_pursuit_stub) | external (controls team's node publishes /drive)
  monitor       true: judge the scenario, write a result, then shut everything down
                false: keep running (interactive work)
  viz           false | true (foxglove_bridge on port 8765)
  record        "" | <bag name>: record raw sensors + ground truth for offline replay
  host, port    CARLA server (localhost:2000)
  realtime_factor  1.0 = real time, 0 = as fast as the stack keeps up
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BRINGUP = get_package_share_directory("planning_bringup")
LOCALIZATION = get_package_share_directory("planning_localization")
CARLA_PKG = get_package_share_directory("planning_carla")

RECORD_TOPICS = [
    "/sensors/lidar/points",
    "/sensors/scan",
    "/sensors/imu",
    "/sensors/gnss",
    "/vehicle/odom",
    "/ground_truth/odom",
    "/ground_truth/objects",
    "/ground_truth/collision",
    "/perception/detections",
    "/goal_pose",
    "/tf_static",
    "/clock",
]


def scenario_path(name: str) -> str:
    if os.path.isabs(name):
        return name
    return os.path.join(BRINGUP, "config", "scenarios", f"{name}.yaml")


def launch_setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    scenario = scenario_path(arg("scenario"))
    with open(scenario) as f:
        town = yaml.safe_load(f)["/**"]["ros__parameters"]["town"]
    graph_file = arg("graph_file") or os.path.join(BRINGUP, "maps", f"{town}_lane_graph.json")
    if not os.path.exists(graph_file):
        raise RuntimeError(
            f"no lane graph for {town}: run `ros2 run planning_carla export_lane_graph --town {town} "
            f"--out src/av_planning/planning_bringup/maps/{town}_lane_graph.json` and rebuild"
        )
    vehicle = os.path.join(BRINGUP, "config", "vehicles", "carla.yaml")
    sim = {"use_sim_time": True}

    actions = [
        Node(
            package="planning_carla",
            executable="carla_adapter",
            output="screen",
            parameters=[
                scenario,
                {
                    "host": arg("host"),
                    "port": int(arg("port")),
                    "sensors_file": os.path.join(CARLA_PKG, "config", "sensors.yaml"),
                    "realtime_factor": float(arg("realtime_factor")),
                },
            ],
        ),
        Node(  # 2D scan for slam_toolbox and the costmap, from a horizontal slice of the 3D lidar
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            remappings=[("cloud_in", "/sensors/lidar/points"), ("scan", "/sensors/scan")],
            parameters=[
                sim,
                {
                    "target_frame": "base_link",
                    "min_height": 0.5,
                    "max_height": 2.0,
                    "range_min": 1.0,
                    "range_max": 70.0,
                    "angle_increment": 0.0087,
                    "scan_time": 0.05,
                    "use_inf": True,
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(LOCALIZATION, "launch", "localization.launch.py")),
            launch_arguments={
                "platform": "carla",
                "mode": arg("localization"),
                "map_file": arg("map_file"),
                "use_sim_time": "true",
            }.items(),
        ),
        Node(package="planning_world_model", executable="tracker", parameters=[sim]),
        Node(
            package="planning_world_model",
            executable="world_model",
            parameters=[sim, {"costmap_size": 60.0, "costmap_resolution": 0.2, "inflation_radius": 1.2}],
        ),
        Node(
            package="planning_nodes",
            executable="global_planner",
            output="screen",
            parameters=[sim, {"graph_file": graph_file, "closed_loop": False}],
        ),
        Node(
            package="planning_nodes",
            executable="behavior_planner",
            parameters=[
                sim,
                {
                    "speed_limit": 8.0,
                    "lane_half_width": 1.6,
                    "stop_distance": 6.0,
                    "clear_distance": 10.0,
                    "stop_margin": 3.0,
                    "follow_range": 40.0,
                    "min_gap": 5.0,
                    "time_gap": 1.5,
                },
            ],
        ),
        Node(
            package="planning_nodes",
            executable="local_planner",
            parameters=[
                sim,
                {"vehicle_file": vehicle, "horizon_m": 40.0, "resample_ds": 0.5, "closed_loop": False},
            ],
        ),
        Node(package="planning_eval", executable="localization_error", parameters=[sim]),
    ]

    if arg("perception") == "fake":
        actions.append(Node(package="planning_sim", executable="fake_perception", parameters=[sim]))
    if arg("controller") == "stub":
        actions.append(
            Node(
                package="planning_sim",
                executable="pure_pursuit_stub",
                parameters=[
                    sim,
                    {
                        "vehicle_file": vehicle,
                        "velocity_topic": "/odometry/filtered",
                        "lookahead_min": 4.0,
                        "lookahead_gain": 0.4,
                    },
                ],
            )
        )
    if arg("monitor").lower() == "true":
        monitor = Node(
            package="planning_eval",
            executable="scenario_monitor",
            output="screen",
            parameters=[scenario, sim],
        )
        actions += [
            monitor,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=monitor, on_exit=[EmitEvent(event=Shutdown(reason="scenario done"))]
                )
            ),
        ]
    if arg("viz").lower() == "true":
        actions.append(
            Node(package="foxglove_bridge", executable="foxglove_bridge", parameters=[sim, {"port": 8765}])
        )
    if arg("record"):
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-s", "mcap", "-o", arg("record"), *RECORD_TOPICS],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="town03_route_empty"),
            DeclareLaunchArgument("graph_file", default_value=""),
            DeclareLaunchArgument("localization", default_value="ground_truth"),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("perception", default_value="fake"),
            DeclareLaunchArgument("controller", default_value="stub"),
            DeclareLaunchArgument("monitor", default_value="true"),
            DeclareLaunchArgument("viz", default_value="false"),
            DeclareLaunchArgument("record", default_value=""),
            DeclareLaunchArgument("host", default_value="localhost"),
            DeclareLaunchArgument("port", default_value="2000"),
            DeclareLaunchArgument("realtime_factor", default_value="1.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
