"""
Launch ArduPilot SITL and MAVROS for MAVLink communication.

Run with default arguments:

ros2 launch ardupilot_sitl sitl_mavros_udp.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description to bring up ArduPilot SITL with MAVROS."""

    # Launch ArduPilot SITL
    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl.launch.py",
                    ]
                ),
            ]
        )
    )

    # Launch MAVProxy (optional, for debugging MAVLink connection)
    mavproxy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "mavproxy.launch.py",
                    ]
                ),
            ]
        )
    )

    # Launch MAVROS node which connects to ArduPilot SITL via UDP
    # MAVROS will:
    # - Listen on UDP port 14551 for incoming data
    # - Send commands to port 14550 where SITL is listening  
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            {
                "fcu_url": "udp://127.0.0.1:14550@127.0.0.1:14551",
            }
        ],
    )

    return LaunchDescription(
        [
            sitl,
            mavproxy,
            mavros_node,
        ]
    )
