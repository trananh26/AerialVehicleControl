"""
Launch MAVROS connected to a real ArduPilot board via serial/USB with RViz visualization.

Uses the official APM plugin configuration matching ros2 launch mavros apm.launch behavior.

Run with default settings (USB serial, Pixhawk):

ros2 launch ardupilot_sitl mavros_real_board.launch.py

Run with custom serial port and baudrate:

ros2 launch ardupilot_sitl mavros_real_board.launch.py fcu_url:=/dev/ttyUSB0:921600

Common ArduPilot serial connections:
- /dev/ttyACM0:57600 (Pixhawk 4 via USB, standard baud)
- /dev/ttyUSB0:57600 (FTDI serial adapter)
- /dev/ttyUSB0:921600 (higher baudrate for faster connection)
"""
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for MAVROS connected to real ArduPilot board with RViz."""
    
    # Get package directories
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_mavros = get_package_share_directory("mavros")
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    
    # Declare launch arguments
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyACM0:57600",
        description="FCU (Flight Control Unit) connection URL. "
                    "Format: /dev/ttyXXX:BAUDRATE or tcp://IP:PORT or udp://IP:PORT@IP:PORT"
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url",
        default_value="",
        description="GCS connection URL (optional)"
    )
    
    tgt_system_arg = DeclareLaunchArgument(
        "tgt_system",
        default_value="1",
        description="Target system ID of the autopilot"
    )
    
    tgt_component_arg = DeclareLaunchArgument(
        "tgt_component",
        default_value="1",
        description="Target component ID"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz visualization."
    )
    
    # Launch RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "iris.rviz"}'],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    
    # Load and publish robot description (simple URDF for RViz display)
    urdf_file = os.path.join(
        pkg_project_bringup, "urdf", "iris_simple.urdf"
    )
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ""},
        ],
    )
    
    # Launch MAVROS node with APM configuration
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace="mavros",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm_pluginlists.yaml"]),
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm_config.yaml"]),
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": LaunchConfiguration("gcs_url"),
                "target_system_id": LaunchConfiguration("tgt_system"),
                "target_component_id": LaunchConfiguration("tgt_component"),
            }
        ],
    )
    
    return LaunchDescription(
        [
            fcu_url_arg,
            gcs_url_arg,
            tgt_system_arg,
            tgt_component_arg,
            use_rviz_arg,
            robot_state_publisher,
            rviz,
            mavros_node,
        ]
    )
