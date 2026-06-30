from setuptools import setup

package_name = "amd_uw_ros2"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/robot_controllers.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AMD-UW",
    maintainer_email="todo@example.com",
    description="ROS2 POC controllers and bridge-side helpers for the AMD-UW Chrono demo.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "constant_speed_controller = amd_uw_ros2.constant_speed_controller:main",
            "manipulator_controller = amd_uw_ros2.manipulator_controller:main",
            "pure_pursuit_controller = amd_uw_ros2.pure_pursuit_controller:main",
            "simple_goal_controller = amd_uw_ros2.simple_goal_controller:main",
        ],
    },
)
