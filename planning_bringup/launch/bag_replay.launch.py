"""Re-run localization and the world model on a recorded bag, scored against ground truth.

  ros2 launch planning_bringup bag_replay.launch.py bag:=/data/bags/gym_levine_01 platform:=gym

Only raw sensor topics are replayed, so the EKF and SLAM you are testing produce
/odometry/filtered, /map and the map->odom->base_link transforms fresh.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BRINGUP = get_package_share_directory("planning_bringup")
LOCALIZATION = get_package_share_directory("planning_localization")


SENSOR_TOPICS = [
    "/sensors/scan",
    "/sensors/imu",
    "/vehicle/odom",
    "/sensors/gnss",
    "/perception/detections",
    "/ground_truth/odom",
    "/tf_static",
    "/clock",
]


def generate_launch_description():
    sim_time = {"use_sim_time": True}
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag"),
            DeclareLaunchArgument("platform", default_value="gym"),
            DeclareLaunchArgument("slam_mode", default_value="mapping"),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("rate", default_value="1.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(LOCALIZATION, "launch", "localization.launch.py")),
                launch_arguments={
                    "platform": LaunchConfiguration("platform"),
                    "slam_mode": LaunchConfiguration("slam_mode"),
                    "map_file": LaunchConfiguration("map_file"),
                    "use_sim_time": "true",
                }.items(),
            ),
            Node(
                package="planning_world_model",
                executable="world_model",
                output="screen",
                parameters=[sim_time],
            ),
            Node(
                package="planning_eval",
                executable="localization_error",
                output="screen",
                parameters=[sim_time],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    LaunchConfiguration("bag"),
                    "--clock",
                    "--rate",
                    LaunchConfiguration("rate"),
                    "--topics",
                    *SENSOR_TOPICS,
                ],
                output="screen",
            ),
        ]
    )
