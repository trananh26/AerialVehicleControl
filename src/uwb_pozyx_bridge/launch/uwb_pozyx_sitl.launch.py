"""Launch the full UWB + Pozyx + ArduPilot SITL pipeline.

This ties together everything that the individual Phase 5 steps verified one
piece at a time:

* the Gazebo world with the four UWB anchors
* the x500 vehicle, spawned at the centre of the anchor square
* the gz -> ROS 2 bridge that carries the ranging data
* the Pozyx bridge node that turns the ranges into beacon frames
* ArduPilot SITL, with the beacon parameters

Run with::

    ros2 launch uwb_pozyx_bridge uwb_pozyx_sitl.launch.py

The vehicle spawns at (5, 5, 0.1), which is the beacon origin, so the beacon
frames are relative to the spawn point.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

#: The beacon origin, which is also where the vehicle spawns.
SPAWN_X = 5.0
SPAWN_Y = 5.0
SPAWN_Z = 0.1

#: The serial port SITL listens on for the Pozyx protocol.
BEACON_SERIAL_PORT = 5761

#: The MAVLink port SITL exposes.
MAVLINK_PORT = 5760


def _spawn_vehicle(context: LaunchContext):
    """Spawn the x500 at the beacon origin.

    The pose has to sit on the ``<include>`` element rather than on the
    model, because ``x500_base/model.sdf`` carries its own ``<pose>`` which
    would otherwise override the requested position.  See docs R14.
    """
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    # The SDF travels inside a protobuf text-format message, so the XML
    # attributes use single quotes to avoid having to escape them.
    sdf = (
        "<sdf version='1.9'>"
        "<model name='x500'>"
        "<include>"
        "<uri>model://x500_base</uri>"
        f"<pose>{x} {y} {z} 0 0 {yaw}</pose>"
        "</include>"
        "</model>"
        "</sdf>"
    )

    return [
        ExecuteProcess(
            cmd=[
                "gz",
                "service",
                "-s",
                "/world/uwb_runway/create",
                "--reqtype",
                "gz.msgs.EntityFactory",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                f"name: 'x500', sdf: \"{sdf}\"",
            ],
            output="screen",
        )
    ]


def _launch_gazebo(context: LaunchContext):
    """Start the Gazebo server with the UWB world.

    The world is addressed by its full path: ``gz sim -s`` looks for a file
    of that name, and the world's share directory is not searched
    recursively.
    """
    world = LaunchConfiguration("world").perform(context)
    world_path = os.path.join(
        get_package_share_directory("uwb_gz_plugin"), "worlds", f"{world}.sdf"
    )

    return [
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "-s", world_path],
            output="screen",
        )
    ]


def _launch_sitl(context: LaunchContext):
    """Start ArduPilot SITL, listening for beacon data on SERIAL1.

    The serial argument has to be exactly ``tcp:<port>:wait``: SITL parses
    the device string as ``tcp:<args1>:<args2>``, so a host in the middle
    shifts everything and the port ends up as base_port + 127.
    """
    pkg_bridge = get_package_share_directory("uwb_pozyx_bridge")
    serial_port = LaunchConfiguration("serial_port").perform(context)
    defaults = os.path.join(pkg_bridge, "config", "beacon.parm")

    return [
        ExecuteProcess(
            cmd=[
                [
                    "arducopter ",
                    "--model x500 ",
                    "--speedup 1 ",
                    "--slave 0 ",
                    "--sim-address=127.0.0.1 ",
                    f"--defaults {defaults} ",
                    f"--serial1=tcp:{serial_port}:wait ",
                    "--synthetic-clock ",
                ]
            ],
            shell=True,
            output="screen",
        )
    ]


def _launch_monitor(context: LaunchContext):
    """Start the MAVLink monitor, which applies the beacon parameters.

    SITL's MAVLink port only serves a single TCP client, so the monitor
    cannot stay connected while another tool wants the same stream.  It
    therefore runs for a bounded time and then exits, freeing the port.
    """
    pkg_bridge = get_package_share_directory("uwb_pozyx_bridge")
    duration = LaunchConfiguration("monitor_duration").perform(context)

    return [
        ExecuteProcess(
            cmd=[
                [
                    "python3 -u ",
                    os.path.join(pkg_bridge, "tools", "sitl_beacon_monitor.py"),
                    f" --mavlink-port {MAVLINK_PORT} --duration {duration}",
                    " --set-params",
                ]
            ],
            shell=True,
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""

    serial_host = LaunchConfiguration("serial_host")
    serial_port = LaunchConfiguration("serial_port")
    use_monitor = LaunchConfiguration("use_monitor")
    monitor_duration = LaunchConfiguration("monitor_duration")

    # --- the gz -> ROS 2 bridge -------------------------------------------
    gz_bridge = Node(
        package="uwb_gz_plugin",
        executable="uwb_gz_to_ros_bridge",
        name="uwb_gz_to_ros_bridge",
        output="screen",
    )

    # --- the Pozyx bridge node --------------------------------------------
    pozyx_node = Node(
        package="uwb_pozyx_bridge",
        executable="pozyx_node",
        name="pozyx_node",
        output="screen",
        parameters=[
            {"input_topic": "/uwb/ranging_ros"},
            {"output_pose_topic": "/uwb/pose"},
            {"serial_host": serial_host},
            {"serial_port": serial_port},
            {"send_rate": 20.0},
            # Metres, not millimetres: see the R2 workaround in the docs.
            {"position_accuracy_m": 0.05},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="uwb_runway",
                description="The Gazebo world to load.",
            ),
            DeclareLaunchArgument(
                "x",
                default_value=str(SPAWN_X),
                description="Spawn x, in metres.",
            ),
            DeclareLaunchArgument(
                "y",
                default_value=str(SPAWN_Y),
                description="Spawn y, in metres.",
            ),
            DeclareLaunchArgument(
                "z",
                default_value=str(SPAWN_Z),
                description="Spawn z, in metres.",
            ),
            DeclareLaunchArgument(
                "yaw",
                default_value="0.0",
                description="Spawn yaw, in radians.",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value=str(BEACON_SERIAL_PORT),
                description="The TCP port SITL listens on for beacon data.",
            ),
            DeclareLaunchArgument(
                "serial_host",
                default_value="127.0.0.1",
                description="The host the beacon node connects to.",
            ),
            DeclareLaunchArgument(
                "use_monitor",
                default_value="True",
                description="Run the MAVLink monitor that sets the parameters.",
            ),
            DeclareLaunchArgument(
                "monitor_duration",
                default_value="45",
                description=(
                    "How long the MAVLink monitor stays connected, in seconds. "
                    "SITL's MAVLink port serves one client at a time, so the "
                    "monitor has to release it before another tool connects."
                ),
            ),
            OpaqueFunction(function=_launch_gazebo),
            # Gazebo needs a moment before it accepts entity creation, and
            # SITL has to be listening on SERIAL1 before the node connects.
            TimerAction(
                period=6.0, actions=[OpaqueFunction(function=_spawn_vehicle)]
            ),
            OpaqueFunction(function=_launch_sitl),
            gz_bridge,
            TimerAction(
                period=8.0,
                actions=[
                    OpaqueFunction(
                        function=_launch_monitor, condition=IfCondition(use_monitor)
                    )
                ],
            ),
            # The node connects to SITL's SERIAL1, which is only bound once
            # SITL has finished starting up; starting earlier just drops
            # the first frames.
            TimerAction(period=14.0, actions=[pozyx_node]),
        ]
    )
