"""EKF + SLAM for any platform.

Arguments:
  platform:     gym | f1tenth | carla        -> picks config/ekf_<platform>.yaml
  slam_mode:    mapping | localization | off  (off = identity map->odom, for testing the EKF alone)
  map_file:     serialized slam_toolbox map, without extension (localization mode)
  use_sim_time: true | false

Who publishes which transform:
  EKF        -> odom -> base_link   (smooth, drifts)
  slam       -> map  -> odom        (drift correction)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    platform = LaunchConfiguration("platform").perform(context)
    slam_mode = LaunchConfiguration("slam_mode").perform(context)
    map_file = LaunchConfiguration("map_file").perform(context)
    sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    share = get_package_share_directory("planning_localization")
    cfg = lambda name: os.path.join(share, "config", name)  # noqa: E731

    nodes = [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[cfg(f"ekf_{platform}.yaml"), {"use_sim_time": sim_time}],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        )
    ]
    if slam_mode == "mapping":
        nodes.append(
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[cfg("slam_mapping.yaml"), {"use_sim_time": sim_time}],
            )
        )
    elif slam_mode == "localization":
        if not map_file:
            raise RuntimeError("slam_mode:=localization needs map_file:=/path/to/map (no extension)")
        nodes.append(
            Node(
                package="slam_toolbox",
                executable="localization_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    cfg("slam_localization.yaml"),
                    {"use_sim_time": sim_time, "map_file_name": map_file},
                ],
            )
        )
    elif slam_mode == "off":
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=["--frame-id", "map", "--child-frame-id", "odom"],
                parameters=[{"use_sim_time": sim_time}],
            )
        )
    else:
        raise RuntimeError(f"unknown slam_mode {slam_mode}")
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("platform", default_value="gym"),
            DeclareLaunchArgument("slam_mode", default_value="mapping"),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
