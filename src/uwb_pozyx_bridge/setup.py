from setuptools import find_packages, setup

package_name = "uwb_pozyx_bridge"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test", "tools"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/x500_uwb.parm"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/x500_uwb_runway.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/tools",
            [
                "tools/sitl_extnav_monitor.py",
                "tools/check_extnav_e2e.py",
                "tools/check_extnav_tracking.py",
                "tools/compare_positions.py",
                "tools/inspect_frames.py",
                "tools/fake_ranging_source.py",
                "tools/verify_fdm_coupling.py",
                "tools/cleanup_stack.sh",
            ],
        ),
    ],
    install_requires=["setuptools", "numpy", "pymavlink"],
    zip_safe=True,
    maintainer="an",
    maintainer_email="an@todo.todo",
    description=(
        "Bridges Gazebo UWB ranging to ArduPilot's external navigation "
        "source via MAVLink ODOMETRY messages."
    ),
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "odometry_node = uwb_pozyx_bridge.odometry_node:main",
        ],
    },
)
