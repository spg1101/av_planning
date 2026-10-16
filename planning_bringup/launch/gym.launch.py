"""Full stack on f1tenth_gym_ros. Start the gym first, in its own terminal or container:

  ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  ros2 launch planning_bringup gym.launch.py waypoints_file:=/path/to/levine.csv

slam_mode: mapping (default) | localization (needs map_file) | off (EKF only, ground-truth-free)
two_agent: true starts fake perception from the opponent car's odometry.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BRINGUP = get_package_share_directory("planning_bringup")
LOCALIZATION = get_package_share_directory("planning_localization")


def generate_launch_description():
    vehicle = os.path.join(BRINGUP, "config", "vehicles", "f1tenth.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints_file", default_value=os.path.join(BRINGUP, "maps", "example_oval.csv")
            ),
            DeclareLaunchArgument("slam_mode", default_value="mapping"),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("two_agent", default_value="false"),
            Node(package="planning_sim", executable="gym_adapter", output="screen"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(LOCALIZATION, "launch", "localization.launch.py")),
                launch_arguments={
                    "platform": "gym",
                    "slam_mode": LaunchConfiguration("slam_mode"),
                    "map_file": LaunchConfiguration("map_file"),
                }.items(),
            ),
            Node(
                package="planning_sim",
                executable="fake_perception",
                output="screen",
                condition=IfCondition(LaunchConfiguration("two_agent")),
            ),
            Node(package="planning_world_model", executable="tracker", output="screen"),
            Node(package="planning_world_model", executable="world_model", output="screen"),
            Node(
                package="planning_nodes",
                executable="global_planner",
                output="screen",
                parameters=[{"waypoints_file": LaunchConfiguration("waypoints_file"), "closed_loop": True}],
            ),
            Node(package="planning_nodes", executable="behavior_planner", output="screen"),
            Node(
                package="planning_nodes",
                executable="local_planner",
                output="screen",
                parameters=[{"vehicle_file": vehicle, "closed_loop": True}],
            ),
            Node(
                package="planning_sim",
                executable="pure_pursuit_stub",
                output="screen",
                parameters=[{"vehicle_file": vehicle, "velocity_topic": "/odometry/filtered"}],
            ),
            Node(package="planning_eval", executable="localization_error", output="screen"),
        ]
    )
