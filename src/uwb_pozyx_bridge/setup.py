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
                "tools/fake_ranging_source.py",
                "tools/compare_odometry_frames.py",
                "tools/check_ekf_heading.py",
                "tools/ab_test_yaw_source.py",
                "tools/ab_run_all_cases.py",
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
