"""Launch the x500 UWB + Pozyx pipeline in Gazebo, RViz and ArduPilot SITL.

This builds on ``ardupilot_gz_bringup/x500_runway.launch.py`` rather than
reimplementing it, so the Gazebo GUI, RViz, the Gazebo/ROS bridges, MAVROS
and MAVProxy all keep working.  On top of that it adds:

* the UWB world with the four anchors (``uwb_runway.sdf``)
* the gz -> ROS 2 bridge that carries the ranging data
* the Pozyx bridge node that turns the ranges into beacon frames
* the MAVLink monitor that applies the beacon parameters

The vehicle spawns at (5, 5, 0.1), which is the beacon origin, so the beacon
frames are relative to the spawn point.

Run with::

    ros2 launch uwb_pozyx_bridge x500_uwb_runway.launch.py

Mission Planner connects to the MAVProxy UDP output on port 14551.
"""

import os
import socket

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node

#: The beacon origin, which is also where the vehicle spawns.
SPAWN_X = 5.0
SPAWN_Y = 5.0
SPAWN_Z = 0.1

#: The serial port SITL listens on for the Pozyx protocol.
BEACON_SERIAL_PORT = 5761


def _uwb_world_path() -> str:
    """Full path to the UWB world."""
    return os.path.join(
        get_package_share_directory("uwb_gz_plugin"), "worlds", "uwb_runway.sdf"
    )


def _merged_params() -> str:
    """Full path to the merged SITL parameter file."""
    return os.path.join(
        get_package_share_directory("uwb_pozyx_bridge"), "config", "x500_uwb.parm"
    )


def _port_is_free(port: int) -> bool:
    """Whether the given TCP port can still be bound."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", port))
        except OSError:
            return False
    return True


def _check_ports(context):
    """Fail early when a port of the stack is still in use.

    SITL binds its MAVLink port (5760) and the beacon port (5761) and
    exits with "bind failed on port 5760 - Address already in use" when
    they are taken.  That message is easy to miss, and the symptom that
    follows is MAVProxy retrying forever with "link 1 down", which looks
    like a telemetry problem rather than a stale process.
    """
    busy = [port for port in (5760, BEACON_SERIAL_PORT) if not _port_is_free(port)]
    if busy:
        raise RuntimeError(
            "TCP port(s) "
            + ", ".join(str(p) for p in busy)
            + " are still in use.  A previous SITL instance is probably "
            "still running.  Stop it first, for example with:\n"
            "    pkill -f arducopter\n"
            "    pkill -f mavproxy.py"
        )
    return []


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    pkg_bringup = get_package_share_directory("ardupilot_gz_bringup")

    # The base stack: Gazebo server + GUI, the x500 with SITL, RViz and the
    # Gazebo/ROS bridges.  Everything the plain runway launch does is kept.
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_bringup, "launch", "x500_runway.launch.py"]
            )
        ),
        launch_arguments={
            "world": _uwb_world_path(),
            "x": str(SPAWN_X),
            "y": str(SPAWN_Y),
            "z": str(SPAWN_Z),
            # The merged parameter file replaces the x500 defaults, because
            # it already contains the x500 tuning plus the beacon settings.
            "defaults": _merged_params(),
            # MAVROS is off by default: it is only needed once the vehicle
            # is actually controlled over ROS.
            "use_mavros": LaunchConfiguration("use_mavros"),
            # SERIAL1 carries the Pozyx beacon protocol.  The ":wait" suffix
            # makes SITL block until the Pozyx node connects, so no beacon
            # frame is missed during start-up.
            "serial1": f"tcp:{BEACON_SERIAL_PORT}:wait",
        }.items(),
    )

    # The gz -> ROS 2 bridge that carries the ranging data.
    gz_bridge = Node(
        package="uwb_gz_plugin",
        executable="uwb_gz_to_ros_bridge",
        name="uwb_gz_to_ros_bridge",
        output="screen",
    )

    # The Pozyx bridge node: ranging -> trilateration -> beacon frames.
    pozyx_node = Node(
        package="uwb_pozyx_bridge",
        executable="pozyx_node",
        name="pozyx_node",
        output="screen",
        parameters=[
            {"input_topic": "/uwb/ranging_ros"},
            {"output_pose_topic": "/uwb/pose"},
            {"serial_host": "127.0.0.1"},
            {"serial_port": BEACON_SERIAL_PORT},
            {"send_rate": 20.0},
            # Metres, not millimetres: see the R2 workaround in the docs.
            {"position_accuracy_m": 0.05},
        ],
    )

    # The MAVLink monitor, which applies the beacon parameters at runtime.
    #
    # It is off by default, because the beacon parameters already live in
    # the merged SITL defaults file and are therefore applied on the very
    # first boot.  Running it is also disruptive: SITL's MAVLink port
    # serves a single TCP client at a time, so the monitor competes with
    # MAVProxy for port 5760 and knocks the GCS link down.
    monitor = OpaqueFunction(
        function=_launch_monitor,
        condition=IfCondition(LaunchConfiguration("use_monitor")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mavros",
                default_value="false",
                description=(
                    "Launch MAVROS.  Off by default; enable it when the "
                    "vehicle has to be controlled over ROS."
                ),
            ),
            DeclareLaunchArgument(
                "use_monitor",
                default_value="false",
                description=(
                    "Run the MAVLink monitor that sets the parameters.  Not "
                    "needed by default: the parameters are in the SITL "
                    "defaults file.  Enabling it takes port 5760 away from "
                    "MAVProxy, so the GCS link drops."
                ),
            ),
            DeclareLaunchArgument(
                "monitor_duration",
                default_value="45",
                description=(
                    "How long the MAVLink monitor stays connected, in seconds."
                ),
            ),
            # Fail early with a clear message when a port of the stack is
            # still taken by a previous run.
            OpaqueFunction(function=_check_ports),
            base,
            gz_bridge,
            # The monitor connects to SITL's MAVLink port, which is only
            # bound once SITL has finished starting up.
            TimerAction(period=8.0, actions=[monitor]),
            # The Pozyx node connects to SITL's SERIAL1, which is only bound
            # once SITL has finished starting up; starting earlier just drops
            # the first frames.
            TimerAction(period=14.0, actions=[pozyx_node]),
        ]
    )


def _launch_monitor(context):
    """Start the MAVLink monitor, which applies the beacon parameters."""
    pkg_bridge = get_package_share_directory("uwb_pozyx_bridge")
    duration = LaunchConfiguration("monitor_duration").perform(context)

    return [
        ExecuteProcess(
            cmd=[
                "python3",
                "-u",
                os.path.join(pkg_bridge, "tools", "sitl_beacon_monitor.py"),
                "--mavlink-port",
                "5760",
                "--duration",
                duration,
                "--set-params",
            ],
            output="screen",
        )
    ]
