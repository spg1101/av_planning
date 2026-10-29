"""Localization for any platform.

Arguments:
  platform:     carla | f1tenth            -> config/ekf_<platform>.yaml, config/slam_<platform>.yaml
  mode:         ground_truth | ekf_only | slam_mapping | slam_localization
  map_file:     serialized slam_toolbox map, without extension (slam_localization)
  use_sim_time: true | false

What each mode publishes (only one node may publish each transform):
  mode               map -> odom              odom -> base_link     needs
  ground_truth       identity (static)        ground truth          /ground_truth/odom
  ekf_only           identity (static)        EKF                   wheel odom + IMU
  slam_mapping       slam_toolbox             EKF                   + /sensors/scan
  slam_localization  slam_toolbox (saved map) EKF                   + /sensors/scan + map_file

In CARLA the adapter sends the true start pose to the EKF (/set_pose) before the first tick,
so the odom and SLAM map frames start aligned with CARLA's world frame (and the lane graph).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODES = ("ground_truth", "ekf_only", "slam_mapping", "slam_localization")


def launch_setup(context):
    platform = LaunchConfiguration("platform").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    map_file = LaunchConfiguration("map_file").perform(context)
    sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower() == "true"}
    if mode not in MODES:
        raise RuntimeError(f"mode must be one of {MODES}, got {mode}")
    share = get_package_share_directory("planning_localization")
    cfg = lambda name: os.path.join(share, "config", name)  # noqa: E731
    identity_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        parameters=[sim_time],
    )

    if mode == "ground_truth":
        return [
            identity_map_to_odom,
            Node(
                package="planning_localization",
                executable="ground_truth_localization",
                output="screen",
                parameters=[sim_time],
            ),
        ]

    nodes = [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[cfg(f"ekf_{platform}.yaml"), sim_time],
            remappings=[("odometry/filtered", "/odometry/filtered"), ("set_pose", "/set_pose")],
        )
    ]
    if mode == "ekf_only":
        nodes.append(identity_map_to_odom)
    elif mode == "slam_mapping":
        nodes.append(
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[cfg(f"slam_{platform}.yaml"), sim_time, {"mode": "mapping"}],
            )
        )
    else:  # slam_localization
        if not map_file:
            raise RuntimeError("mode:=slam_localization needs map_file:=/path/to/map (no extension)")
        nodes.append(
            Node(
                package="slam_toolbox",
                executable="localization_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    cfg(f"slam_{platform}.yaml"),
                    sim_time,
                    {"mode": "localization", "map_file_name": map_file, "map_start_at_dock": True},
                ],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("platform", default_value="carla"),
            DeclareLaunchArgument("mode", default_value="ground_truth"),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
