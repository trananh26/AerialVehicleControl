# Copyright 2026 AerialVehicleControl.
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Step 5: end-to-end UWB navigation with ArduPilot SITL.

Launches, in order:

  1. ArduPilot SITL (``arducopter``) on TCP 5760, with the
     UWB-specific parameter file.
  2. MAVROS, connected to SITL via TCP, with the ``vision_pose`` plugin
     enabled so ``/mavros/vision_pose/pose`` becomes a MAVLink
     ``VISION_POSITION_ESTIMATE`` consumed by the EKF.
  3. The gz → ROS bridge + trilateration + relay (the uwb_bridge
     pipeline) so the SITL actually receives a pose.

Note: this launch assumes a Gazebo world is already running and
publishing UWB range messages on the gz transport topics the
``uwb_gz_to_ros_bridge`` subscribes to.  See the repo README for the
gz command to use.
"""

import shlex

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_sitl_process(context, *args, **kwargs):
    """Spawn ``arducopter`` as a child process with the right defaults."""
    sitl_bin = LaunchConfiguration("sitl_bin").perform(context)
    model = LaunchConfiguration("model").perform(context)
    speedup = LaunchConfiguration("speedup").perform(context)
    home = LaunchConfiguration("home").perform(context)
    parm_file = LaunchConfiguration("parm_file").perform(context)
    tcp_port = LaunchConfiguration("sitl_tcp_port").perform(context)

    cmd = [
        sitl_bin,
        "--model", model,
        "--speedup", speedup,
        "--home", home,
        "--defaults", parm_file,
        # TCP listen so MAVROS can connect in via tcpout://
        "--uartA=tcp:" + tcp_port + ":wait",
    ]
    return [
        ExecuteProcess(
            cmd=cmd,
            name="ardupilot_sitl",
            output="screen",
        ),
    ]


def _build_mavros_process(context, *args, **kwargs):
    """Wrap ``mavros_node`` in bash so MAVROS_PLUGIN_LIBS is exported."""
    fcu_url = LaunchConfiguration("fcu_url").perform(context)
    gcs_url = LaunchConfiguration("gcs_url").perform(context)
    pluginlists = LaunchConfiguration("pluginlists_yaml").perform(context)
    config = LaunchConfiguration("config_yaml").perform(context)

    inner = (
        "exec ros2 run mavros mavros_node --ros-args "
        f"-p fcu_url:={shlex.quote(fcu_url)} "
        f"-p gcs_url:={shlex.quote(gcs_url)} "
        f"-p pluginlists_yaml:={shlex.quote(pluginlists)} "
        f"-p config_yaml:={shlex.quote(config)} "
        "-p tgt_system:=1 -p tgt_component:=1 "
        "-r __ns:=/mavros"
    )
    cmd = [
        "/bin/bash", "-c",
        "export MAVROS_PLUGIN_LIBS="
        "'libmavros_plugins.so;libmavros_extras_plugins.so' && " + inner,
    ]
    return [
        ExecuteProcess(
            cmd=cmd,
            name="mavros",
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_dir = "uwb_bridge"

    return LaunchDescription([
        # Make the UWB pipeline output visible to MAVROS in the same
        # DDS domain.
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),

        DeclareLaunchArgument(
            "sitl_bin",
            default_value="/home/an/ros2_project/AerialVehicleControl/"
                          "src/ardupilot/build/sitl/bin/arducopter",
            description="ArduPilot SITL binary path.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="+",
            description="SITL frame model ('+' = default multirotor).",
        ),
        DeclareLaunchArgument(
            "speedup",
            default_value="1",
            description="SITL wall-clock speedup factor.",
        ),
        DeclareLaunchArgument(
            "home",
            default_value="0,0,0,0",
            description="SITL home lat,lon,alt,yaw (NED origin).",
        ),
        DeclareLaunchArgument(
            "parm_file",
            default_value=[
                "$(find ", pkg_dir, ")/config/copter-uwb.parm",
            ],
            description="Path to the UWB parameter file.",
        ),
        DeclareLaunchArgument(
            "sitl_tcp_port",
            default_value="5760",
            description="TCP port SITL listens on for MAVROS.",
        ),

        DeclareLaunchArgument(
            "fcu_url",
            default_value="tcp://127.0.0.1:5760",
            description="MAVROS → SITL TCP URL.",
        ),
        DeclareLaunchArgument(
            "gcs_url",
            default_value="udp://@127.0.0.1:14550",
            description="Optional GCS URL.",
        ),
        DeclareLaunchArgument(
            "pluginlists_yaml",
            default_value=[
                "$(find ", pkg_dir, ")/config/mavros_pluginlists.yaml",
            ],
            description="MAVROS plugin list (empty denylist for extras).",
        ),
        DeclareLaunchArgument(
            "config_yaml",
            default_value=[
                "$(find ", pkg_dir, ")/config/mavros_vision.yaml",
            ],
            description="MAVROS per-plugin config (vision_pose).",
        ),

        # 1. ArduPilot SITL
        OpaqueFunction(function=_build_sitl_process),

        # 2. The gz → ROS pipeline + relay (no MAVLink bridge; MAVROS
        #    replaces it).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare(pkg_dir), "/launch/uwb_bridge.launch.py",
            ]),
            launch_arguments={
                "enable_mavros": "false",          # we start our own MAVROS
                "enable_mavlink_bridge": "false",  # pymavlink bypass
            }.items(),
        ),

        # 3. MAVROS with vision_pose plugin
        OpaqueFunction(function=_build_mavros_process),
    ])
