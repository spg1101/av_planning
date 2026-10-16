"""Planning stack on CARLA. Placeholder until the CARLA adapter exists (integration owner).

The adapter must publish the standard topics (/sensors/*, /vehicle/odom, /clock,
/ground_truth/odom) and consume /drive. CARLA must run in synchronous mode.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BRINGUP = get_package_share_directory("planning_bringup")
LOCALIZATION = get_package_share_directory("planning_localization")


def generate_launch_description():
    vehicle = os.path.join(BRINGUP, "config", "vehicles", "carla.yaml")
    sim_time = {"use_sim_time": True}
    return LaunchDescription(
        [
            DeclareLaunchArgument("waypoints_file"),
            DeclareLaunchArgument("slam_mode", default_value="mapping"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(LOCALIZATION, "launch", "localization.launch.py")),
                launch_arguments={
                    "platform": "carla",
                    "slam_mode": LaunchConfiguration("slam_mode"),
                    "use_sim_time": "true",
                }.items(),
            ),
            Node(package="planning_world_model", executable="tracker", parameters=[sim_time]),
            Node(package="planning_world_model", executable="world_model", parameters=[sim_time]),
            Node(
                package="planning_nodes",
                executable="global_planner",
                parameters=[
                    sim_time,
                    {"waypoints_file": LaunchConfiguration("waypoints_file"), "closed_loop": False},
                ],
            ),
            Node(
                package="planning_nodes",
                executable="behavior_planner",
                parameters=[sim_time, {"speed_limit": 8.0, "lane_half_width": 1.6, "stop_distance": 8.0}],
            ),
            Node(
                package="planning_nodes",
                executable="local_planner",
                parameters=[sim_time, {"vehicle_file": vehicle, "horizon_m": 40.0, "closed_loop": False}],
            ),
            Node(package="planning_eval", executable="localization_error", parameters=[sim_time]),
        ]
    )
