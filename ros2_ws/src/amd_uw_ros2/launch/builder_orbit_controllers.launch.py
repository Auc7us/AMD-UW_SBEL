"""Launch circular path controllers for tracked builders."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def parse_builder_ids(builder_ids_text):
    builder_ids = []
    for raw_part in builder_ids_text.split(","):
        part = raw_part.strip()
        if not part:
            continue
        builder_id = int(part)
        if builder_id <= 0:
            raise ValueError(
                "builder_ids must contain positive ranks only"
            )
        builder_ids.append(builder_id)
    if not builder_ids:
        raise ValueError("builder_ids must contain at least one rank")
    return builder_ids


def launch_setup(context, *args, **kwargs):
    builder_ids = parse_builder_ids(
        LaunchConfiguration("builder_ids").perform(context)
    )
    float_args = {
        name: ParameterValue(
            LaunchConfiguration(name), value_type=float
        )
        for name in (
            "center_x",
            "center_y",
            "work_circle_radius_m",
            "path_radius_m",
            "target_speed_mps",
            "lookahead_m",
        )
    }
    counter_clockwise = ParameterValue(
        LaunchConfiguration("counter_clockwise"), value_type=bool
    )

    return [
        Node(
            package="amd_uw_ros2",
            executable="builder_orbit_controller",
            name=f"builder_{builder_id}_orbit_controller",
            output="screen",
            parameters=[
                {
                    "builder_id": builder_id,
                    "counter_clockwise": counter_clockwise,
                    **float_args,
                }
            ],
        )
        for builder_id in builder_ids
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "builder_ids",
                default_value="1,2",
                description="Comma-separated builder ranks.",
            ),
            DeclareLaunchArgument("center_x", default_value="0.0"),
            DeclareLaunchArgument("center_y", default_value="0.0"),
            DeclareLaunchArgument(
                "work_circle_radius_m", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "path_radius_m", default_value="40.0"
            ),
            DeclareLaunchArgument(
                "target_speed_mps", default_value="1.0"
            ),
            DeclareLaunchArgument("lookahead_m", default_value="8.0"),
            DeclareLaunchArgument(
                "counter_clockwise", default_value="true"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
