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
    drop_band_half_width_m = ParameterValue(
        LaunchConfiguration("drop_band_half_width_m"), value_type=float
    )
    drop_arc_tolerance_m = ParameterValue(
        LaunchConfiguration("drop_arc_tolerance_m"), value_type=float
    )
    home_slowdown_distance_m = ParameterValue(
        LaunchConfiguration("home_slowdown_distance_m"), value_type=float
    )
    approach_arc_m = ParameterValue(LaunchConfiguration("approach_arc_m"), value_type=float)
    rear_reference_offset_m = ParameterValue(LaunchConfiguration("rear_reference_offset_m"), value_type=float)
    pickup_request_rate_hz = ParameterValue(LaunchConfiguration("pickup_request_rate_hz"), value_type=float)
    skip_failed_targets = ParameterValue(LaunchConfiguration("skip_failed_targets"), value_type=bool)
    command_timeout_s = ParameterValue(LaunchConfiguration("command_timeout_s"), value_type=float)
    command_timeout_sim_s = ParameterValue(LaunchConfiguration("command_timeout_sim_s"), value_type=float)
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
                        "drop_band_half_width_m": drop_band_half_width_m,
                        "drop_arc_tolerance_m": drop_arc_tolerance_m,
                        "home_slowdown_distance_m": home_slowdown_distance_m,
                        "approach_arc_m": approach_arc_m,
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
                        "command_timeout_sim_s": command_timeout_sim_s,
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
                "drop_band_half_width_m",
                default_value="2.0",
                description=(
                    "Half-width of the radial band around the collector circle that "
                    "counts as arrived at the drop point, so a band twice this wide. "
                    "The drop point is not a surveyed spot -- rocks only have to land "
                    "near this rank's builder -- and demanding a tight circle made "
                    "rovers orbit a point they were already standing beside."
                ),
            ),
            DeclareLaunchArgument(
                "drop_arc_tolerance_m",
                default_value="1.0",
                description=(
                    "How far along the collector circle the rover may stop from its "
                    "drop point. This is a BIAS, not a tolerance: arrival is accepted "
                    "the instant the rover is inside the band, so it parks this far "
                    "short every time (measured 2.73-2.90 m at 3.0). It can be tight "
                    "because the run-in arrives ALONG the circle and in_drop_band also "
                    "accepts on a stop line, so undershooting cannot strand it. This "
                    "MUST match the node default in pure_pursuit_controller.py -- the "
                    "launch value wins, so changing only the node changes nothing."
                ),
            ),
            DeclareLaunchArgument(
                "approach_arc_m",
                default_value="12.0",
                description=(
                    "Arc of collector circle the rover follows into its drop point, so "
                    "it arrives running ALONG the circumference instead of nose-in at "
                    "it. The rear-discharging trailer then pours a line of rock along "
                    "the circle rather than a heap across it."
                ),
            ),
            DeclareLaunchArgument(
                "home_slowdown_distance_m",
                default_value="25.0",
                description=(
                    "Distance over which the return leg tapers from cruise down to "
                    "home_approach_speed_mps, measured to the DROP BAND BOUNDARY. "
                    "Mirrors pickup_slowdown_offset_m so arriving at the drop point "
                    "is as gentle as arriving at a rock; a short taper meant braking "
                    "from ~4.5 m/s and throwing the load out of the bed."
                ),
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
                default_value="1800.0",
                description=(
                    "WALL-clock backstop only, for a sim that is not progressing at all. "
                    "It is not a duration budget: the sim runs ~19x slower than real time and "
                    "that ratio moves with rank count, terrain and machine, so a wall number "
                    "cannot bound arm work. The old 120 s default was ~6 s of sim and fired on "
                    "essentially every SUCCESSFUL grab. Use command_timeout_sim_s instead."
                ),
            ),
            DeclareLaunchArgument(
                "command_timeout_sim_s",
                default_value="90.0",
                description="Manipulator command timeout in SIMULATION seconds (the real deadline).",
            ),
            DeclareLaunchArgument(
                "arm_cmd_republish_rate_hz",
                default_value="1.0",
                description="Rate for resending arm commands until the C++ arm bridge acknowledges them.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
