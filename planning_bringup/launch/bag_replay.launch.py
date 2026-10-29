"""Re-run localization and the world model on a recorded CARLA bag, scored against ground truth.

Record once in CARLA:   ros2 launch planning_bringup carla.launch.py record:=t03_lap1 monitor:=false
Replay many times:      ros2 launch planning_bringup bag_replay.launch.py bag:=t03_lap1 mode:=slam_mapping

Only raw sensor topics are replayed, so the EKF and SLAM under test produce /odometry/filtered,
/map and the map->odom->base_link transforms fresh. No CARLA needed: Member A can tune
localization on a laptop, and every run sees exactly the same data.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOCALIZATION = get_package_share_directory("planning_localization")

SENSOR_TOPICS = [
    "/sensors/lidar/points",
    "/sensors/scan",
    "/sensors/imu",
    "/sensors/gnss",
    "/vehicle/odom",
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
            DeclareLaunchArgument("platform", default_value="carla"),
            DeclareLaunchArgument("mode", default_value="slam_mapping"),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("rate", default_value="1.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(LOCALIZATION, "launch", "localization.launch.py")),
                launch_arguments={
                    "platform": LaunchConfiguration("platform"),
                    "mode": LaunchConfiguration("mode"),
                    "map_file": LaunchConfiguration("map_file"),
                    "use_sim_time": "true",
                }.items(),
            ),
            Node(
                package="planning_localization",
                executable="initial_pose_from_ground_truth",
                parameters=[sim_time],
            ),
            Node(package="planning_world_model", executable="world_model", parameters=[sim_time]),
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
                    "--delay",
                    "2.0",
                    "--topics",
                    *SENSOR_TOPICS,
                ],
                output="screen",
            ),
        ]
    )
