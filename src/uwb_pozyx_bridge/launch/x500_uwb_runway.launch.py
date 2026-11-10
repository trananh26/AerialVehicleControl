"""Launch the x500 UWB + external navigation pipeline in Gazebo, RViz and ArduPilot SITL.

This builds on ``ardupilot_gz_bringup/x500_runway.launch.py`` rather than
reimplementing it, so the Gazebo GUI, RViz, the Gazebo/ROS bridges, MAVROS
and MAVProxy all keep working.  On top of that it adds:

* the UWB world with the four anchors (``uwb_runway.sdf``), rendered for the
  layout that is requested on the command line
* the gz -> ROS 2 bridge that carries the ranging data
* the odometry bridge node that turns the ranges into MAVLink ODOMETRY
  messages for ArduPilot's external navigation (EXTNAV) source
* the MAVLink monitor that applies the parameters

The vehicle spawns at a random point inside the anchor rectangle, so the
ranging geometry is not the degenerate "directly under an anchor" case on
every run.  The spawn point is expressed in the Gazebo world frame (ENU);
the anchor frame of the solver is the same rectangle with ``x = North`` and
``y = East``.

The layout is a rectangle, not necessarily a square: ``layout_size_x_m`` is
the length of the ``a0 -> a1`` side and ``layout_size_y_m`` the length of
the ``a0 -> a3`` side.  The defaults match the ESP32 tag's
``CFG_DEFAULT_LAYOUT_SIZE_X_M`` / ``CFG_DEFAULT_LAYOUT_SIZE_Y_M``.

Run with::

    ros2 launch uwb_pozyx_bridge x500_uwb_runway.launch.py

Mission Planner connects to the MAVProxy UDP output on port 14551.
"""

import os
import random
import socket
import tempfile

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

from uwb_pozyx_bridge.uwb_layout import (
    LAYOUT_SIZE_X_M,
    LAYOUT_SIZE_Y_M,
    AnchorLayout,
)
from uwb_pozyx_bridge.uwb_world import write_world

#: The height of the anchor plane above the ground, in metres.
ANCHOR_PLANE_HEIGHT_M = 2.0

#: The height of the vehicle above the ground at spawn time, in metres.
SPAWN_Z = 0.2

#: The serial port SITL listens on for the MAVLink external navigation link.
NAV_SERIAL_PORT = 5761

#: The name of the Gazebo world, which has to match the <world name="...">
#: attribute in uwb_runway.sdf.
WORLD_NAME = "uwb_runway"

#: The global reference of the UWB world.  ArduPilot needs a global origin
#: for its EKF even when the position comes from the UWB ranging data,
#: which is relative to the anchors.  These values match the origin that
#: the Gazebo world is placed at.
ORIGIN_LATITUDE_DEG = -35.363262
ORIGIN_LONGITUDE_DEG = 149.165237
ORIGIN_ALTITUDE_M = 584.0


def _uwb_world_template_path() -> str:
    """Full path to the UWB world template."""
    return os.path.join(
        get_package_share_directory("uwb_gz_plugin"), "worlds", "uwb_runway.sdf"
    )


def _merged_params() -> str:
    """Full path to the merged SITL parameter file."""
    return os.path.join(
        get_package_share_directory("uwb_pozyx_bridge"), "config", "x500_uwb.parm"
    )


def _layout(context) -> AnchorLayout:
    """Build the layout from the launch arguments."""
    return AnchorLayout(
        size_x_m=float(LaunchConfiguration("layout_size_x_m").perform(context)),
        size_y_m=float(LaunchConfiguration("layout_size_y_m").perform(context)),
        alpha_deg=float(LaunchConfiguration("layout_alpha_deg").perform(context)),
        plane_height_m=ANCHOR_PLANE_HEIGHT_M,
    )


def _render_world(context) -> str:
    """Render the Gazebo world for the requested layout.

    The world file of the package is the template; the anchor positions are
    substituted so that the plugin, the visible anchor models and the solver
    all describe the same rectangle.

    Returns:
        The path of the rendered world file.
    """
    layout = _layout(context)
    template_path = _uwb_world_template_path()
    with open(template_path, "r", encoding="utf-8") as handle:
        template = handle.read()

    target = os.path.join(
        tempfile.gettempdir(),
        f"uwb_runway_{layout.size_x_m:.3f}x{layout.size_y_m:.3f}"
        f"_a{layout.alpha_deg:.3f}.sdf",
    )
    write_world(target, template, layout)
    return target


def _spawn_position(context) -> tuple[float, float]:
    """Pick the spawn point of the vehicle.

    The point is drawn uniformly inside the anchor rectangle, with a
    margin to the edges so that the ranging geometry is never degenerate.
    With ``random_spawn`` disabled the centre of the rectangle is used,
    which is the reproducible default.

    Returns:
        The ``(east_m, north_m)`` spawn position in the Gazebo world frame.
    """
    layout = _layout(context)

    if LaunchConfiguration("random_spawn").perform(context).lower() not in (
        "true",
        "1",
        "yes",
    ):
        east, north = layout.centre_enu()
        return (east, north)

    seed = int(LaunchConfiguration("spawn_seed").perform(context) or 0)
    rng = random.Random(seed if seed else None)
    return layout.random_spawn_enu(rng)


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

    SITL binds its MAVLink port (5760) and the navigation port (5761) and
    exits with "bind failed on port 5760 - Address already in use" when
    they are taken.  That message is easy to miss, and the symptom that
    follows is MAVProxy retrying forever with "link 1 down", which looks
    like a telemetry problem rather than a stale process.
    """
    busy = [port for port in (5760, NAV_SERIAL_PORT) if not _port_is_free(port)]
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


def _base_stack(context):
    """Include the base stack, with the spawn point resolved.

    The spawn point cannot be given as a launch argument of this file
    directly, because the base launch file takes it as a string and the
    value has to be computed here (a random point inside the rectangle).
    """
    spawn_east, spawn_north = _spawn_position(context)
    world_path = _render_world(context)
    pkg_bringup = get_package_share_directory("ardupilot_gz_bringup")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_bringup, "launch", "x500_runway.launch.py"]
                )
            ),
            launch_arguments={
                "world": world_path,
                "x": f"{spawn_east:.3f}",
                "y": f"{spawn_north:.3f}",
                "z": f"{SPAWN_Z:.3f}",
                # The merged parameter file replaces the x500 defaults, because
                # it already contains the x500 tuning plus the external
                # navigation settings.
                "defaults": _merged_params(),
                # MAVROS is off by default: it is only needed once the vehicle
                # is actually controlled over ROS.
                "use_mavros": LaunchConfiguration("use_mavros"),
                # The Gazebo world of this launch file is the UWB world, so the
                # bridge has to subscribe to /world/uwb_runway/... and not to
                # /world/runway/...  Without this the joint states never arrive
                # and RViz reports "No transform from [rotor_0] to [odom]".
                "world_name": WORLD_NAME,
                # SERIAL1 carries the MAVLink external navigation link.  The
                # ":wait" suffix makes SITL block until the odometry node
                # connects, so no fix is missed during start-up.
                "serial1": f"tcp:{NAV_SERIAL_PORT}:wait",
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    # The gz -> ROS 2 bridge that carries the ranging data.
    gz_bridge = Node(
        package="uwb_gz_plugin",
        executable="uwb_gz_to_ros_bridge",
        name="uwb_gz_to_ros_bridge",
        output="screen",
    )

    # The odometry bridge node: ranging -> trilateration -> ODOMETRY.
    odometry_node = Node(
        package="uwb_pozyx_bridge",
        executable="odometry_node",
        name="odometry_node",
        output="screen",
        parameters=[
            {"input_topic": "/uwb/ranging_ros"},
            {"output_pose_topic": "/uwb/pose"},
            {"serial_host": "127.0.0.1"},
            {"serial_port": NAV_SERIAL_PORT},
            {"send_rate": 20.0},
            # The layout of the anchors.  ``layout_alpha_deg`` is the bearing
            # of the direction ``a0 -> a1`` measured from North; it is the
            # same quantity the ESP32 tag calls ``config_yaw_offset_deg``.
            # The layout is a rectangle: the two sides are independent.
            {"layout_size_x_m": LaunchConfiguration("layout_size_x_m")},
            {"layout_size_y_m": LaunchConfiguration("layout_size_y_m")},
            {"layout_alpha_deg": LaunchConfiguration("layout_alpha_deg")},
            {"anchor_plane_height_m": ANCHOR_PLANE_HEIGHT_M},
            # Standard deviation of the position estimate, in metres.  It
            # is squared to obtain the covariance entries of the ODOMETRY
            # message, from which ArduPilot derives the EKF measurement
            # noise.
            {"position_std_m": 0.05},
            # The global reference of the UWB world.  ArduPilot needs it to
            # set the EKF origin: the ranging data is relative to the
            # anchors, so the autopilot cannot derive the origin by itself.
            # Without an origin it rejects MAV_CMD_NAV_TAKEOFF and the
            # vehicle arms but never leaves the ground.
            {"origin_latitude_deg": ORIGIN_LATITUDE_DEG},
            {"origin_longitude_deg": ORIGIN_LONGITUDE_DEG},
            {"origin_altitude_m": ORIGIN_ALTITUDE_M},
        ],
    )

    # The MAVLink monitor, which applies the parameters at runtime.
    #
    # It is off by default, because the parameters already live in the
    # merged SITL defaults file and are therefore applied on the very
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
                default_value="true",
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
            DeclareLaunchArgument(
                "layout_size_x_m",
                default_value=f"{LAYOUT_SIZE_X_M}",
                description=(
                    "Length of the a0 -> a1 side of the anchor rectangle, in "
                    "metres.  Matches CFG_DEFAULT_LAYOUT_SIZE_X_M on the ESP32."
                ),
            ),
            DeclareLaunchArgument(
                "layout_size_y_m",
                default_value=f"{LAYOUT_SIZE_Y_M}",
                description=(
                    "Length of the a0 -> a3 side of the anchor rectangle, in "
                    "metres.  Matches CFG_DEFAULT_LAYOUT_SIZE_Y_M on the ESP32."
                ),
            ),
            DeclareLaunchArgument(
                "layout_alpha_deg",
                default_value="0.0",
                description=(
                    "Bearing of the direction a0 -> a1, measured from North "
                    "and positive towards the East, in degrees."
                ),
            ),
            DeclareLaunchArgument(
                "random_spawn",
                default_value="true",
                description=(
                    "Spawn the vehicle at a random point inside the anchor "
                    "rectangle.  Set to false to spawn it at the centre."
                ),
            ),
            DeclareLaunchArgument(
                "spawn_seed",
                default_value="0",
                description=(
                    "Seed of the random spawn.  Zero means a fresh random "
                    "point on every run."
                ),
            ),
            # Fail early with a clear message when a port of the stack is
            # still taken by a previous run.
            OpaqueFunction(function=_check_ports),
            OpaqueFunction(function=_base_stack),
            gz_bridge,
            # The monitor connects to SITL's MAVLink port, which is only
            # bound once SITL has finished starting up.
            TimerAction(period=8.0, actions=[monitor]),
            # The odometry node connects to SITL's SERIAL1, which is only
            # bound once SITL has finished starting up; starting earlier just
            # drops the first fixes.
            TimerAction(period=14.0, actions=[odometry_node]),
        ]
    )


def _launch_monitor(context):
    """Start the MAVLink monitor, which applies the parameters."""
    pkg_bridge = get_package_share_directory("uwb_pozyx_bridge")
    duration = LaunchConfiguration("monitor_duration").perform(context)

    return [
        ExecuteProcess(
            cmd=[
                "python3",
                "-u",
                os.path.join(pkg_bridge, "tools", "sitl_extnav_monitor.py"),
                "--mavlink-port",
                "5760",
                "--duration",
                duration,
                "--set-params",
            ],
            output="screen",
        )
    ]
