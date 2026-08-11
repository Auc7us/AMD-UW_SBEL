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
            "waypoint_spacing_m",
            "lookahead_m",
            "steering_kp",
            "steering_ki",
            "radial_kp_rad_per_m",
            "hull_bias_curvature_per_m",
            "steering_limit",
            "speed_kp",
            "max_throttle",
            "station_radius_tol_m",
            "station_radius_release_m",
            "station_tolerance_rad",
            "station_keep_deadband_m",
            "station_keep_speed_mps",
            "station_keep_min_speed_mps",
            "station_keep_max_steering",
        )
    }
    counter_clockwise = ParameterValue(
        LaunchConfiguration("counter_clockwise"), value_type=bool
    )

    arm_cycle_enabled = ParameterValue(
        LaunchConfiguration("arm_cycle_enabled"), value_type=bool
    )
    # The arm controller needs NO site geometry. Every target it solves is read off the
    # simulation (arm_base_pose, pick_target, place_target), which is what makes it robust
    # to a tracked vehicle stopping wherever it actually stops on sloped regolith. It only
    # needs to agree with BuilderRig about how long the arm is.
    arm_float_args = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name in ("arm_scale",)
    }

    nodes = []
    for builder_id in builder_ids:
        nodes.append(
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
        )
        # The two nodes are a pair and neither works alone. The arm controller lays a rock
        # and the simulation advances the station angle; the orbit controller creeps the
        # ~1 m to the next slot and parks; only then does the sim mark the next pick
        # ready. Launch only the drive half and the builder holds slot 0 forever.
        nodes.append(
            Node(
                package="amd_uw_ros2",
                executable="builder_arm_controller",
                name=f"builder_{builder_id}_arm_controller",
                output="screen",
                parameters=[
                    {
                        "builder_id": builder_id,
                        "cycle_enabled": arm_cycle_enabled,
                        **arm_float_args,
                    }
                ],
            )
        )
    return nodes


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
                "path_radius_m", default_value="33.0"
            ),
            # Only the initial 2.5 m drive from spawn to slot 0 uses this; after that the
            # builder creeps ~1 m per rock under station keeping. Dropped from 1.0 because
            # a tracked vehicle does not stop on a point, and every metre of overshoot on
            # acquisition is a metre of standing lane offset for the rest of the course.
            DeclareLaunchArgument(
                "target_speed_mps", default_value="0.9"
            ),
            # Aggressive on purpose: steering brakes a track, so the speed loop is fighting
            # the steering, and brake-steer authority scales with speed. A timid speed loop
            # lets the hull slow until it can no longer turn, which is self-reinforcing.
            DeclareLaunchArgument("speed_kp", default_value="2.0"),
            DeclareLaunchArgument("max_throttle", default_value="1.0"),
            DeclareLaunchArgument("arm_cycle_enabled", default_value="true"),
            # Must match BuilderRig's arm_geometry_scale.
            DeclareLaunchArgument("arm_scale", default_value="2.0"),
            DeclareLaunchArgument("waypoint_spacing_m", default_value="2.0"),
            # Arc ahead on the lane to steer at. Shrinks automatically as the builder
            # closes on its station, so this is the CRUISING value.
            # SHORT on purpose. The hull turns a 10.5 m radius unsteered against a 33 m
            # lane, so authority is not the limit -- letting the error grow is. A long
            # lookahead lets the hull slip on the way to a distant target, so corrections
            # are applied to a stale situation and the loop closes around a growing
            # excursion until the steering rails.
            DeclareLaunchArgument("lookahead_m", default_value="2.5"),
            DeclareLaunchArgument("steering_kp", default_value="1.2"),
            # Integral action on heading error. NOT optional: measured open-loop, this
            # hull yaws +0.119 rad/s at zero steering while a 33 m lane needs 0.038, so it
            # requires a standing steering trim of about -0.21 that a proportional-only
            # law cannot hold. See steering_ki in builder_orbit_controller.py.
            DeclareLaunchArgument("steering_ki", default_value="0.9"),
            # Cross-track gain: heading given up per metre of radial error off the lane.
            DeclareLaunchArgument("radial_kp_rad_per_m", default_value="0.35"),
            # Measured natural curvature of the hull with zero steering: 0.0952 1/m, a
            # 10.5 m turn radius to the left, from asymmetric track drag. Feedforwarding
            # it is what lets the builder hold a 33 m lane at all. Set to 0.0 to disable
            # the feedforward and see the inward spiral it prevents.
            DeclareLaunchArgument("hull_bias_curvature_per_m", default_value="0.0952"),
            # Never let the command reach the rail: full steering locks a track, and a
            # locked track means the hull pivots instead of translating, so it can no
            # longer correct its path. This cap is what keeps it driving.
            DeclareLaunchArgument("steering_limit", default_value="0.5"),
            # How far off the lane still counts as on-station. Oscillation about the lane
            # is expected and harmless; the arm's reach is what actually constrains it.
            DeclareLaunchArgument("station_radius_tol_m", default_value="0.9"),
            # Radial band at which an already-held station is GIVEN UP. Wider than the
            # band to take one, because dropping station keeping also releases the
            # sim-side anchor that pulls the hull back onto the lane -- so a single band
            # turns the drift that follows parking into a lap round the site. That is
            # what had rank 3 laying one rock to its neighbours' four.
            DeclareLaunchArgument("station_radius_release_m", default_value="1.05"),
            # Station keeping, retuned for the build cycle. The station now steps ONE WALL
            # SLOT at a time -- 0.9 m of course, 0.99 m of lane -- where it used to jump 30
            # degrees per harvest cycle, so the old bands are the wrong size by an order of
            # magnitude:
            #   tolerance 0.05 rad is 1.65 m of arc, so a whole build step fell INSIDE the
            #     "already arrived" band and the builder never registered leaving station.
            #   deadband 0.4 m stopped the creep 0.4 m short of a 0.99 m step, i.e. 40% of
            #     the rock pitch, which is most of a rock's width of error in the wall.
            # The tolerance only gates ENTRY into station keeping; the deadband does the
            # precision, so the tolerance can stay loose enough that a moving builder
            # cannot step over the band between control ticks and commit to another lap.
            # 0.015 rad is 0.5 m of arc -- ten 20 Hz samples at the drive speed below.
            #
            # The deadband is what the builder actually parks within, and it must be a
            # distance an M113 can stop inside. 0.12 m was too tight to be reachable at
            # all: with the creep speed floored (below), the builder now overshoots into
            # the band and brakes rather than stalling just outside it. Overshoot is
            # cheap here -- wall slots are fixed world points, so being a little along
            # the lane costs reach margin, not placement accuracy.
            DeclareLaunchArgument("station_tolerance_rad", default_value="0.015"),
            DeclareLaunchArgument("station_keep_deadband_m", default_value="0.25"),
            DeclareLaunchArgument("station_keep_speed_mps", default_value="0.35"),
            # Floor on the creep speed. Without it the commanded speed tapers to a few
            # mm/s just outside the deadband, the M113 does not move at all, and the
            # builder never signals that it is on station -- so the arm is never offered
            # a pick. This is what stalled every builder in the 4-rank run.
            DeclareLaunchArgument("station_keep_min_speed_mps", default_value="0.25"),
            # Steering cap during the fine approach: a skid-steer with steering but no
            # throttle pivots in place and skids sideways off the lane.
            DeclareLaunchArgument("station_keep_max_steering", default_value="0.35"),
            DeclareLaunchArgument(
                "counter_clockwise", default_value="true"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
