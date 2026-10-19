# =============================================================================
# uwb_bridge — Python ROS 2 package
# =============================================================================
import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = "uwb_bridge"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        # Launch files
        (os.path.join("share", PACKAGE_NAME, "launch"),
            glob("launch/*.launch.py")),
        # Config files (YAML + ArduPilot param files)
        (os.path.join("share", PACKAGE_NAME, "config"),
            glob("config/*.yaml") + glob("config/*.parm")),
    ],
    install_requires=["setuptools", "pymavlink", "numpy"],
    zip_safe=True,
    maintainer="AerialVehicleControl",
    maintainer_email="an@todo.todo",
    description=(
        "Trilateration + ROS 2 bridge for the UWB simulation plugin."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "uwb_trilateration_node = "
            "uwb_bridge.uwb_trilateration_node:main",
            "mavlink_bridge_node = "
            "uwb_bridge.mavlink_bridge_node:main",
            "uwb_to_mavros_vision_node = "
            "uwb_bridge.uwb_to_mavros_vision_node:main",
        ],
    },
)
