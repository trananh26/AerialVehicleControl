# Copyright 2026 AerialVehicleControl.
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Launch the UWB bridge stack: ros_gz_bridge + trilateration + MAVLink."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_dir = "uwb_bridge"

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_mavros",
            default_value="true",
            description=(
                "Start MAVROS with the vision_pose_estimate plugin "
                "and the uwb_to_mavros_vision relay."
            ),
        ),
        DeclareLaunchArgument(
            "enable_mavlink_bridge",
            default_value="true",
            description="Start the MAVLink bridge node (requires pymavlink).",
        ),
        DeclareLaunchArgument(
            "gz_bridge_config",
            default_value=[
                "$(find ", pkg_dir, ")/config/ros_gz_bridge.yaml"
            ],
            description="Path to ros_gz_bridge YAML config.",
        ),
        DeclareLaunchArgument(
            "mavlink_url",
            default_value="udpout:127.0.0.1:14550",
            description="MAVLink output (matches ardupilot_sitl default).",
        ),

        # ---- gz <-> ROS 2 bridge ----
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "--ros-args", "-p", "config_file:=",
                LaunchConfiguration("gz_bridge_config"),
            ],
            name="ros_gz_bridge",
            output="screen",
        ),

        # ---- trilateration ----
        Node(
            package=pkg_dir,
            executable="uwb_trilateration_node",
            name="uwb_trilateration_node",
            output="screen",
            parameters=[{
                # Match the C++ bridge output topic (best-effort QoS).
                "input_topic": "/uwb/ranging/ros",
                "output_pose_topic": "/uwb/pose",
                "output_covariance_topic": "/uwb/pose_cov",
                "frame_id": "map",
                "child_frame_id": "uwb_tag",
            }],
        ),

        # ---- relay: /uwb/pose_cov -> /mavros/vision_pose/pose ----
        Node(
            package=pkg_dir,
            executable="uwb_to_mavros_vision_node",
            name="uwb_to_mavros_vision_node",
            output="screen",
            parameters=[{
                "input_topic": "/uwb/pose_cov",
                "output_topic": "/mavros/vision_pose/pose",
                "frame_id": "map",
            }],
        ),

        # ---- MAVROS + vision_pose plugin (optional) ----
        OpaqueFunction(function=_maybe_start_mavros),

        # ---- MAVLink bridge (optional, bypasses MAVROS) ----
        OpaqueFunction(function=_maybe_start_mavlink),
    ])


def _maybe_start_mavros(context, *args, **kwargs):
    """Conditionally start MAVROS with the vision_pose_estimate plugin."""
    enable = LaunchConfiguration("enable_mavros").perform(context)
    if enable.lower() not in ("true", "1", "yes"):
        return []
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    FindPackageShare("uwb_bridge"),
                    "/launch/mavros_vision.launch.py",
                ]
            )
        ),
    ]


def _maybe_start_mavlink(context, *args, **kwargs):
    """Conditionally start the MAVLink node based on launch arg."""
    enable = LaunchConfiguration("enable_mavlink_bridge").perform(context)
    if enable.lower() not in ("true", "1", "yes"):
        return []

    url = LaunchConfiguration("mavlink_url").perform(context)
    node = Node(
        package="uwb_bridge",
        executable="mavlink_bridge_node",
        name="mavlink_bridge_node",
        output="screen",
        parameters=[{
            "pose_topic": "/uwb/pose_cov",
            "mavlink_url": url,
            "send_rate_hz": 25.0,
        }],
    )
    return [node]
