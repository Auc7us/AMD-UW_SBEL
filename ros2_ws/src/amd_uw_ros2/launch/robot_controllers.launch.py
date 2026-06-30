"""Launch drive and manipulator controllers for one or more AMD-UW robots."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def parse_robot_ids(robot_ids_text):
    robot_ids = []
    for raw_part in robot_ids_text.split(","):
        part = raw_part.strip()
        if not part:
            continue
        robot_id = int(part)
        if robot_id <= 0:
            raise ValueError("robot_ids must contain positive robot ranks only")
        robot_ids.append(robot_id)

    if not robot_ids:
        raise ValueError("robot_ids must contain at least one robot rank")

    return robot_ids


def launch_setup(context, *args, **kwargs):
    robot_ids = parse_robot_ids(LaunchConfiguration("robot_ids").perform(context))

    target_speed_mps = ParameterValue(LaunchConfiguration("target_speed_mps"), value_type=float)
    switch_radius_m = ParameterValue(LaunchConfiguration("switch_radius_m"), value_type=float)
    rock_side_offset_m = ParameterValue(LaunchConfiguration("rock_side_offset_m"), value_type=float)
    rear_reference_offset_m = ParameterValue(LaunchConfiguration("rear_reference_offset_m"), value_type=float)
    pickup_request_rate_hz = ParameterValue(LaunchConfiguration("pickup_request_rate_hz"), value_type=float)
    skip_failed_targets = ParameterValue(LaunchConfiguration("skip_failed_targets"), value_type=bool)
    command_timeout_s = ParameterValue(LaunchConfiguration("command_timeout_s"), value_type=float)
    arm_cmd_republish_rate_hz = ParameterValue(
        LaunchConfiguration("arm_cmd_republish_rate_hz"),
        value_type=float,
    )

    nodes = []
    for robot_id in robot_ids:
        nodes.append(
            Node(
                package="amd_uw_ros2",
                executable="pure_pursuit_controller",
                name=f"robot_{robot_id}_pure_pursuit",
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                        "target_speed_mps": target_speed_mps,
                        "switch_radius_m": switch_radius_m,
                        "rock_side_offset_m": rock_side_offset_m,
                        "rear_reference_offset_m": rear_reference_offset_m,
                        "pickup_request_rate_hz": pickup_request_rate_hz,
                    }
                ],
            )
        )
        nodes.append(
            Node(
                package="amd_uw_ros2",
                executable="manipulator_controller",
                name=f"robot_{robot_id}_manipulator",
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                        "skip_failed_targets": skip_failed_targets,
                        "command_timeout_s": command_timeout_s,
                        "arm_cmd_republish_rate_hz": arm_cmd_republish_rate_hz,
                    }
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ids",
                default_value="1",
                description="Comma-separated robot ranks to control, for example: 1,2",
            ),
            DeclareLaunchArgument(
                "target_speed_mps",
                default_value="1.0",
                description="Cruise speed for each pure-pursuit controller.",
            ),
            DeclareLaunchArgument(
                "switch_radius_m",
                default_value="1.0",
                description="Pickup wait radius around each selected rock.",
            ),
            DeclareLaunchArgument(
                "rock_side_offset_m",
                default_value="1.5",
                description="Lateral drive waypoint offset from each rock.",
            ),
            DeclareLaunchArgument(
                "rear_reference_offset_m",
                default_value="1.25",
                description="Rear reference point offset behind the tractor center.",
            ),
            DeclareLaunchArgument(
                "pickup_request_rate_hz",
                default_value="1.0",
                description="Rate for republishing pickup requests while waiting.",
            ),
            DeclareLaunchArgument(
                "skip_failed_targets",
                default_value="true",
                description="Mark failed manipulator targets done so driving can continue.",
            ),
            DeclareLaunchArgument(
                "command_timeout_s",
                default_value="120.0",
                description="Manipulator command timeout before reporting failure.",
            ),
            DeclareLaunchArgument(
                "arm_cmd_republish_rate_hz",
                default_value="1.0",
                description="Rate for resending arm commands until the C++ arm bridge acknowledges them.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
