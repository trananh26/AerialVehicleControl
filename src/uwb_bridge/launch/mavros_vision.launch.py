# Copyright 2026 AerialVehicleControl.
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Launch MAVROS with the ``vision_pose_estimate`` plugin enabled.

Designed to be included from ``uwb_bridge.launch.py`` (or run directly).

Note: this launch starts MAVROS even if no FC is reachable. The plugin
will keep retrying the connection; the relay node
(``uwb_to_mavros_vision_node``) publishes the pose regardless, which is
useful for ROS-side testing of the pipeline (Step 4 verification).

The MAVROS plugin loader only scans shared libraries listed in the
``MAVROS_PLUGIN_LIBS`` environment variable.  The apt build of MAVROS
hard-codes a value that does **not** include ``libmavros_extras_plugins.so``,
which is where ``vision_pose`` (the plugin that converts
``/mavros/vision_pose/pose`` into a MAVLink ``VISION_POSITION_ESTIMATE``)
lives.  We override the env var via a bash wrapper that exports it
before exec'ing the mavros_node binary; ``SetEnvironmentVariable`` /
``additional_env`` do not propagate through ``ros2 run``.
"""

import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _build_mavros_process(context, *args, **kwargs):
    """Build the ``ExecuteProcess`` that runs ``mavros_node``.

    We wrap the ``ros2 run mavros mavros_node ...`` invocation in a
    ``/bin/bash -c`` so that we can ``export MAVROS_PLUGIN_LIBS`` to
    include ``libmavros_extras_plugins.so`` (which hosts
    ``vision_pose``).
    """
    fcu_url = LaunchConfiguration("fcu_url").perform(context)
    gcs_url = LaunchConfiguration("gcs_url").perform(context)
    pluginlists = LaunchConfiguration("pluginlists_yaml").perform(context)
    config = LaunchConfiguration("config_yaml").perform(context)

    # Build the inner ros2 command with proper shell escaping.
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
        DeclareLaunchArgument(
            "fcu_url",
            default_value="udp://:14550@127.0.0.1:14555",
            description=(
                "MAVROS FCU URL. Default listens on UDP 14550 and connects "
                "out to 14555, matching the ardupilot_sitl defaults."
            ),
        ),
        DeclareLaunchArgument(
            "gcs_url",
            default_value="udp://@127.0.0.1:14550",
            description="Optional GCS URL (QGroundControl can attach here).",
        ),
        DeclareLaunchArgument(
            "pluginlists_yaml",
            default_value=[
                "$(find ", pkg_dir, ")/config/mavros_pluginlists.yaml",
            ],
            description=(
                "MAVROS plugin list.  We use a custom list with empty "
                "denylist so the vision_pose plugin (from mavros_extras) "
                "is allowed to load."
            ),
        ),
        DeclareLaunchArgument(
            "config_yaml",
            default_value=[
                "$(find ", pkg_dir, ")/config/mavros_vision.yaml",
            ],
            description="MAVROS per-plugin config (tunes vision_pose).",
        ),

        OpaqueFunction(function=_build_mavros_process),
    ])
