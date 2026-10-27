from setuptools import find_packages, setup

package_name = "uwb_pozyx_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tools"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/beacon.parm"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="an",
    maintainer_email="an@todo.todo",
    description=(
        "Bridges Gazebo UWB ranging to ArduPilot's Pozyx beacon backend."
    ),
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "pozyx_node = uwb_pozyx_bridge.pozyx_node:main",
        ],
    },
)
