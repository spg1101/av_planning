"""Fastest loop: kinematic sim + the full planning stack + pure pursuit stub. No gym, no GPU.

ros2 launch planning_bringup kinematic.launch.py
ros2 launch planning_bringup kinematic.launch.py perfect_tracking:=true   # planning without controls
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BRINGUP = get_package_share_directory("planning_bringup")
LOCALIZATION = get_package_share_directory("planning_localization")


def generate_launch_description():
    vehicle = os.path.join(BRINGUP, "config", "vehicles", "f1tenth.yaml")
    waypoints = LaunchConfiguration("waypoints_file")
    perfect = LaunchConfiguration("perfect_tracking")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints_file", default_value=os.path.join(BRINGUP, "maps", "example_oval.csv")
            ),
            DeclareLaunchArgument("perfect_tracking", default_value="false"),
            Node(
                package="planning_sim",
                executable="kinematic_sim",
                output="screen",
                parameters=[{"vehicle_file": vehicle, "perfect_tracking": perfect}],
            ),
            Node(
                package="planning_world_model",
                executable="world_model",
                output="screen",
                parameters=[{"velocity_topic": "/vehicle/odom"}],
            ),
            Node(
                package="planning_nodes",
                executable="global_planner",
                output="screen",
                parameters=[{"waypoints_file": waypoints, "closed_loop": True}],
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
                parameters=[{"vehicle_file": vehicle, "velocity_topic": "/vehicle/odom"}],
            ),
        ]
    )
