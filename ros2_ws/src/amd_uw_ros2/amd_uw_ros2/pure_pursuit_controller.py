import math
from dataclasses import dataclass
from typing import List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def approach(current: float, target: float, max_delta: float) -> float:
    if current < target:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class RobotState:
    x: float
    y: float
    yaw: float
    speed: float


@dataclass
class VehicleCommand:
    steering: float = 0.0
    throttle: float = 0.0
    brake: float = 1.0


class PurePursuitController(Node):
    """Drive through /targetPos points with pure-pursuit steering."""

    def __init__(self) -> None:
        super().__init__("pure_pursuit_controller")

        self._hard_turning = False
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("target_speed_mps", 1.0)
        self.declare_parameter("speed_kp", 0.55)
        self.declare_parameter("speed_tolerance_mps", 0.08)
        self.declare_parameter("target_speed_ramp_mps2", 10.0)
        self.declare_parameter("throttle_ramp_per_s", 10.0)
        self.declare_parameter("brake_ramp_per_s", 10.0)
        # Steering slew-rate limit (units/s of the [-1,1] command). Caps how fast the
        # commanded steer angle can change so pure-pursuit corrections come in smoothly
        # instead of snapping. 2.5 => full lock in ~0.4 s. Lower = gentler/slower.
        self.declare_parameter("steering_ramp_per_s", 2.5)
        self.declare_parameter("switch_radius_m", 1.0)
        self.declare_parameter("lookahead_min_m", 2.0)
        # Upper bound on the pure-pursuit lookahead. Without it the lookahead becomes
        # the raw distance to the target, and curvature 2*sin(alpha)/L vanishes for a
        # far goal, so the rover barely steers on a long leg.
        self.declare_parameter("lookahead_max_m", 8.0)
        # Beyond this bearing the target is behind the vehicle, where pure pursuit is
        # degenerate (sin(pi) = 0). Steer at full lock instead until it comes back into
        # the forward arc; the hysteresis band prevents chatter on the boundary.
        self.declare_parameter("reverse_turn_alpha_rad", 1.2)
        self.declare_parameter("reverse_turn_hysteresis_rad", 0.35)
        # A wheeled vehicle CANNOT turn in place: with Ackermann steering at full lock
        # and no forward speed there is no yaw moment, so the tires just plow and the
        # rover sits there digging. Full lock also saturates the lateral tire force on
        # low-traction regolith, which loses steering authority exactly when it is
        # needed. So turn on a bounded arc, under power, with room to swing: there is
        # 20+ m of open ground on the return leg, so a wide arc costs nothing.
        self.declare_parameter("reverse_turn_steering", 0.6)
        self.declare_parameter("reverse_turn_speed_mps", 1.2)
        self.declare_parameter("wheelbase_m", 2.5)
        self.declare_parameter("max_steering_angle_rad", 0.6)
        self.declare_parameter("rock_side_offset_m", 1.5)
        self.declare_parameter("rear_reference_offset_m", 1.25)
        # Grab sector. Clean grabs land at 60.0-60.3 deg, so 55 leaves margin for a
        # small undershoot. The upper bound refuses the abeam case (65-90 deg produced
        # folded, near-singular arm poses), where the only IK solution folds the arm
        # back through the rover.
        self.declare_parameter("pickup_angle_min_deg", 55.0)
        self.declare_parameter("pickup_angle_max_deg", 80.0)
        # Distance over which target speed ramps linearly down to the boundary
        # speed. On deformable SCM soil the rover decelerates much more slowly than
        # on rigid ground, so a 10 m band left it still hot at the boundary -> the
        # speed controller slammed the brake, spiking the front axle and locking the
        # steering. Widened so it actually reaches ~boundary speed by the boundary
        # without hard braking. Increase further if it still enters too fast.
        self.declare_parameter("pickup_slowdown_offset_m", 25.0)
        # Crawl into the pickup zone so a real full stop is reachable: on low-traction
        # lunar terrain a rover entering at 2 m/s just slides with the wheels locked
        # and never settles. Ramp down to a slow boundary speed and allow it to reach
        # ~0 at the boundary (min approach floor = 0).
        self.declare_parameter("pickup_min_approach_speed_mps", 0.0)
        self.declare_parameter("pickup_boundary_speed_mps", 1.0)
        self.declare_parameter("pickup_request_rate_hz", 1.0)
        # End-of-mission return leg: once every rock is collected or skipped, drive
        # back to the spawn point and stop there so the load can be dumped.
        self.declare_parameter("home_tolerance_m", 1.5)
        # Drop band: see in_drop_band(). The site centre is the hub every rank's ray
        # radiates from; the band is concentric with the collector circle that the
        # drop point sits on, so its radius is taken from the drop point itself and
        # does not have to be configured here.
        self.declare_parameter("site_center_x", 0.0)
        self.declare_parameter("site_center_y", 0.0)
        self.declare_parameter("drop_band_half_width_m", 2.0)
        # Was 8 m, because a rover driving STRAIGHT AT the drop point cannot converge on
        # it -- pure pursuit orbits a target inside its own turning radius -- so it had to
        # be accepted from a long way out. The tangential run-in (see home_approach_target)
        # removes that constraint: the rover arrives moving ALONG the circle, straight at
        # the drop point, so it can be held to a much tighter arc. 8 m of slack would now
        # be worse than useless -- the rover would enter the band and park 8 m short of
        # the pile it is meant to be building, right where the entry waypoint puts it.
        # 1.0, not 3.0, and the difference matters more than it looks. Arrival is accepted
        # the INSTANT the rover is inside this band, so the band is not a tolerance -- it is
        # a systematic bias. Measured at 3.0 m, three rovers parked 2.73, 2.77 and 2.90 m
        # short of their drop point, every time, because that is where the band starts. Add
        # the 2.4 m the pour lip trails behind the tractor and the load landed ~5 m
        # clockwise of the slot it was for, outside the arm's envelope.
        #
        # It can be this tight now for the same reason it had to be loose before: the rover
        # arrives ALONG the circle rather than driving at the point, and in_drop_band also
        # accepts on a stop line once committed, so undershooting cannot strand it.
        self.declare_parameter("drop_arc_tolerance_m", 1.0)
        # Tangential approach to the collector circle. The rover comes home from a rock
        # line that runs radially OUTWARD from the drop point, so left to itself it
        # arrives radially -- nose-in at the circumference, with the trailer pointing out
        # into open field and the load tipping wherever the rover happened to stop.
        #
        # Instead it is given two waypoints. The first is on the collector circle,
        # approach_arc_m of arc CLOCKWISE of the drop point (clockwise = behind it, since
        # the lane walks counter-clockwise). The second is the drop point itself, reached
        # by following the circle. By the time the rover has run that arc its heading is
        # tangential, so the rear-discharging trailer pours a line of rock along the
        # circumference rather than a heap across it.
        #
        # approach_arc_m has to be long enough for the heading to settle -- the rover
        # turns at ~5 m radius -- and short enough that it is not a detour. Reaching the
        # entry waypoint is not required to be precise: approach_capture_m accepts it
        # generously, and everything after that is circle-following.
        self.declare_parameter("approach_arc_m", 12.0)
        self.declare_parameter("approach_capture_m", 5.0)
        self.declare_parameter("approach_lookahead_m", 6.0)
        self.declare_parameter("home_approach_speed_mps", 1.0)
        # Matches pickup_slowdown_offset_m, and for the same reason: on this soil the
        # rover sheds speed slowly, so a short taper means arriving hot and braking
        # hard. It was 8 m, measured to the wrong point (see distance_to_drop_band),
        # which put ~4.5 m/s of momentum into a full-brake stop and launched rocks
        # out of the bed.
        self.declare_parameter("home_slowdown_distance_m", 25.0)
        # Only request a pickup once the chassis has actually come to a FULL STOP:
        # the arm IK is solved for the base pose at request time, and the rover
        # coasts after the wheels brake (low lunar traction), so requesting while
        # still moving leaves the gripper short of the rock. "Full stop" = speed at
        # or below stop_speed held continuously for stop_dwell_s (so we fire at rest,
        # not while still decelerating through the threshold). stop_timeout_s is a
        # safety fallback so a rover that can't fully settle still proceeds.
        # All three are SIMULATION seconds now. pickup_stop_timeout_s was 10 wall
        # seconds = 0.5 s of sim, so the "safety fallback" fired on essentially
        # every grab: the logs are full of `TIMEOUT, dwell=0.00s`, i.e. the rover
        # was told to grab while still rolling, every single time, in every run.
        # 10 s of sim is the deadline that was actually intended.
        self.declare_parameter("pickup_stop_speed_mps", 0.05)
        self.declare_parameter("pickup_stop_dwell_s", 0.4)
        self.declare_parameter("pickup_stop_timeout_s", 10.0)
        # Backstop against a target that never resolves. The manipulator is supposed
        # to answer every pickup request with target_done -- success, failure, or
        # skip -- and if it does not, the rover brakes and waits for it forever with
        # no detector firing anywhere: it is stopped on purpose, so it is not STUCK,
        # and it is not diverging either. That is exactly how rank 1 lost the last
        # 265 s of sim in the 2-rank run to a single unsolvable IK. SIM seconds.
        self.declare_parameter("target_done_timeout_sim_s", 180.0)
        self.declare_parameter("post_done_straighten_time_s", 0.75)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.ego_state_topic = f"/robot_{self.robot_id}/egoState"
        self.target_pos_topic = f"/robot_{self.robot_id}/targetPos"
        self.target_done_topic = f"/robot_{self.robot_id}/target_done"
        self.pickup_request_topic = f"/robot_{self.robot_id}/pickup_request"
        self.home_pos_topic = f"/robot_{self.robot_id}/homePos"
        self.mission_done_topic = f"/robot_{self.robot_id}/mission_done"
        self.at_home_topic = f"/robot_{self.robot_id}/at_home"
        self.command_topic = f"/robot_{self.robot_id}/vehicle_cmd"

        self.state: Optional[RobotState] = None
        self.targets: List[Tuple[float, float, float]] = []
        self.target_index = -1
        self.completed_targets: Set[int] = set()
        self.drive_target_index = -1
        self.drive_target_offset = (0.0, 0.0)
        self.have_targets = False
        self.waiting_for_target_done = False
        self.stop_dwell_s = 0.0                    # how long we've been at/below stop_speed
        self.wait_start_s: Optional[float] = None  # when we began waiting at this target
        self.straighten_until_time_s: Optional[float] = None
        self.last_pickup_request_time_s: Optional[float] = None
        self.last_pickup_request_index = -1
        # Spawn point, published by the C++ drive bridge so the site-layout maths
        # is not duplicated here.
        self.home: Optional[Tuple[float, float]] = None
        self.returning_home = False
        self.parked_at_home = False
        self.command = VehicleCommand()
        self.ramped_target_speed = 0.0
        # Breakaway state: see compute_speed_command.
        self._stalled_since: Optional[float] = None
        self._breakaway_active = False
        # Simulation clock, from egoState element 5. See sim_now().
        self.sim_time: Optional[float] = None
        # Speed at which the drop band was entered on this cycle: see drive_home.
        self._drop_entry_speed: Optional[float] = None
        # Latched once the rover has reached the entry waypoint and turned onto the
        # collector circle: see home_approach_target.
        self._approach_committed = False
        # When the rover first got within switch_radius of the current drive target
        # while still outside the pickup angle sector: see on_timer.
        # When the active target was selected. One deadline per target (see
        # target_done_timeout_sim_s) covers every way a rank can stall on one rock:
        # circling it without ever reaching a usable angle, or waiting on a
        # manipulator that never answers.
        self._target_started_s: Optional[float] = None

        self.command_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.pickup_request_pub = self.create_publisher(Float64MultiArray, self.pickup_request_topic, 10)
        self.create_subscription(Float64MultiArray, self.ego_state_topic, self.on_ego_state, 10)
        self.create_subscription(Float64MultiArray, self.target_pos_topic, self.on_target_pos, 10)
        self.create_subscription(Bool, self.target_done_topic, self.on_target_done, 10)
        self.at_home_pub = self.create_publisher(Bool, self.at_home_topic, 10)
        self.create_subscription(Float64MultiArray, self.home_pos_topic, self.on_home_pos, 10)
        self.create_subscription(Bool, self.mission_done_topic, self.on_mission_done, 10)

        rate_hz = max(1e-6, float(self.get_parameter("control_rate_hz").value))
        self.dt = 1.0 / rate_hz
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"robot_{self.robot_id} pure pursuit: {self.ego_state_topic} + "
            f"{self.target_pos_topic} + {self.target_done_topic} -> "
            f"{self.command_topic} + {self.pickup_request_topic}"
        )

    def on_ego_state(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warn("Ignoring egoState message; expected [x, y, yaw, speed].")
            return

        self.state = RobotState(
            x=float(msg.data[0]),
            y=float(msg.data[1]),
            yaw=float(msg.data[2]),
            speed=float(msg.data[3]),
        )
        # Element 5 (index 5) is SIM time. Optional so an older sim still drives.
        if len(msg.data) >= 6:
            self.sim_time = float(msg.data[5])

    def on_target_pos(self, msg: Float64MultiArray) -> None:
        layout_dims = getattr(getattr(msg, "layout", None), "dim", [])
        is_xyz = bool(layout_dims and getattr(layout_dims[0], "label", "") == "xyz")

        if is_xyz:
            if len(msg.data) % 3 != 0:
                self.get_logger().warn("Ignoring targetPos xyz message; expected [x0, y0, z0, ...].")
                return
            targets = [
                (float(msg.data[i]), float(msg.data[i + 1]), float(msg.data[i + 2]))
                for i in range(0, len(msg.data), 3)
            ]
        else:
            if len(msg.data) % 2 != 0:
                self.get_logger().warn("Ignoring targetPos message; expected [x0, y0, x1, y1, ...].")
                return
            targets = [
                (float(msg.data[i]), float(msg.data[i + 1]), 0.0)
                for i in range(0, len(msg.data), 2)
            ]

        if not targets:
            return

        # A GROWING target list means the sim spawned a new harvest cycle: this rank's
        # lane rotated and fresh rocks were put out. End-of-mission state is latched
        # (returning_home / parked_at_home stay set once home is reached), so without
        # clearing it here the rover would sit at the old drop point forever with new
        # rocks on the ground. Indices are stable because the sim APPENDS, so
        # completed_targets stays valid and the new rocks are simply unfinished ones.
        grew = len(targets) > len(self.targets)
        self.targets = targets
        if grew and self.have_targets:
            self.get_logger().info(
                f"targetPos grew to {len(self.targets)}; new harvest cycle -- resuming collection."
            )
            self.returning_home = False
            self.parked_at_home = False
            self.straighten_until_time_s = None
            self._drop_entry_speed = None
            self._approach_committed = False
        if not self.have_targets:
            self.have_targets = True
            self.get_logger().info(f"Received {len(self.targets)} targetPos points.")
        self.completed_targets = {i for i in self.completed_targets if i < len(self.targets)}
        if self.target_index >= len(self.targets):
            self.target_index = -1
            self.drive_target_index = -1
            self.waiting_for_target_done = False

    def on_target_done(self, msg: Bool) -> None:
        if not msg.data:
            return

        if not (0 <= self.target_index < len(self.targets)):
            self.get_logger().info("Ignoring target_done=true; no active targetPos is selected.")
            return

        self.completed_targets.add(self.target_index)
        self.get_logger().info(f"target_done=true; completed targetPos[{self.target_index}], selecting next target.")
        self.target_index = -1
        self.drive_target_index = -1
        self.waiting_for_target_done = False
        self.stop_dwell_s = 0.0
        self.wait_start_s = None
        self.last_pickup_request_time_s = None
        self.last_pickup_request_index = -1
        self.ramped_target_speed = 0.0
        self.command = VehicleCommand(steering=0.0, throttle=0.0, brake=1.0)
        # SIM seconds: this is a physical manoeuvre (straighten the wheels before
        # driving off), so a wall-clock deadline made it ~0.04 s of sim, i.e. nothing.
        self.straighten_until_time_s = (
            self.sim_now() + max(0.0, float(self.get_parameter("post_done_straighten_time_s").value))
        )

    def abandon_current_target(self) -> None:
        """Give up on the active rock and move on, as if it had been skipped.

        Same bookkeeping as on_target_done, but no claim that anything succeeded.
        Marking it completed is deliberate: the rank must not re-select the rock it
        just proved it cannot solve, or it will deadlock on it again immediately.
        """
        if 0 <= self.target_index < len(self.targets):
            self.completed_targets.add(self.target_index)
        self.target_index = -1
        self.drive_target_index = -1
        self.waiting_for_target_done = False
        self.stop_dwell_s = 0.0
        self.wait_start_s = None
        self.last_pickup_request_time_s = None
        self.last_pickup_request_index = -1
        self.ramped_target_speed = 0.0
        self._target_started_s = None
        self.command = VehicleCommand(steering=0.0, throttle=0.0, brake=1.0)

    def on_home_pos(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warn("Ignoring homePos; expected at least [x, y].")
            return
        home = (float(msg.data[0]), float(msg.data[1]))
        if self.home is None or math.hypot(home[0] - self.home[0], home[1] - self.home[1]) > 1e-3:
            # The drop point moved: the lane stepped to the next harvest cycle. The
            # run-in has to be flown again for the new one.
            self._approach_committed = False
        self.home = home

    def on_mission_done(self, msg: Bool) -> None:
        if not msg.data or self.returning_home:
            return
        self.returning_home = True
        self._approach_committed = False
        self.get_logger().info(
            "mission_done=true; every target is collected or skipped, returning to spawn."
        )

    def drop_band_errors(self) -> Optional[Tuple[float, float, float, float]]:
        """(radial_err, arc_err, band_half_width, arc_tolerance) for the drop band.

        Returns None when the band cannot be evaluated. Shared by in_drop_band (is
        the rover there yet) and distance_to_drop_band (how much further), so the
        arrival test and the speed taper can never disagree about where the band is.
        """
        if self.home is None or self.state is None:
            return None
        cx = float(self.get_parameter("site_center_x").value)
        cy = float(self.get_parameter("site_center_y").value)
        band = abs(float(self.get_parameter("drop_band_half_width_m").value))
        arc_tol = abs(float(self.get_parameter("drop_arc_tolerance_m").value))
        if band <= 0.0 or arc_tol <= 0.0:
            return None

        home_radius = math.hypot(self.home[0] - cx, self.home[1] - cy)
        if home_radius < 1e-6:
            return None
        here_radius = math.hypot(self.state.x - cx, self.state.y - cy)
        radial_err = abs(here_radius - home_radius)

        home_angle = math.atan2(self.home[1] - cy, self.home[0] - cx)
        here_angle = math.atan2(self.state.y - cy, self.state.x - cx)
        arc_err = abs(wrap_to_pi(here_angle - home_angle)) * home_radius
        return (radial_err, arc_err, band, arc_tol)

    def home_approach_target(self) -> Tuple[float, float]:
        """Where to steer on the return leg, so the rover arrives ALONG the circle.

        Two stages, latched by _approach_committed so the switch cannot chatter:

          not committed -- steer at the entry waypoint, approach_arc_m of arc clockwise
                           of the drop point on the collector circle. Commit once within
                           approach_capture_m of it, or once the rover is already on the
                           circle somewhere in the arc between it and the drop point.
          committed     -- steer at a point ON the circle, approach_lookahead_m of arc
                           ahead of the rover's own bearing, never past the drop point.
                           That is pure pursuit with the path as the target instead of
                           the goal, which is what makes the final heading tangential
                           rather than radial.

        Falls back to the drop point itself if the geometry cannot be evaluated -- which
        is the old behaviour, so a missing parameter degrades rather than strands.
        """
        if self.home is None or self.state is None:
            return self.home if self.home is not None else (self.state.x, self.state.y)

        cx = float(self.get_parameter("site_center_x").value)
        cy = float(self.get_parameter("site_center_y").value)
        radius = math.hypot(self.home[0] - cx, self.home[1] - cy)
        arc = abs(float(self.get_parameter("approach_arc_m").value))
        if radius < 1e-6 or arc <= 0.0:
            return self.home

        def on_circle(angle: float) -> Tuple[float, float]:
            return (cx + radius * math.cos(angle), cy + radius * math.sin(angle))

        home_angle = math.atan2(self.home[1] - cy, self.home[0] - cx)
        here_angle = math.atan2(self.state.y - cy, self.state.x - cx)
        here_radius = math.hypot(self.state.x - cx, self.state.y - cy)
        # Signed arc from here to the drop point, positive counter-clockwise -- the
        # direction the lane walks, and so the direction the run-in travels.
        arc_to_home = wrap_to_pi(home_angle - here_angle) * radius

        if not self._approach_committed:
            entry = on_circle(home_angle - arc / radius)
            capture = abs(float(self.get_parameter("approach_capture_m").value))
            band = abs(float(self.get_parameter("drop_band_half_width_m").value))
            near_entry = math.hypot(entry[0] - self.state.x, entry[1] - self.state.y) <= capture
            # Already on the circle, in the run-in arc: nothing is gained by driving back
            # to a waypoint the rover has effectively reached.
            in_run_in = 0.0 <= arc_to_home <= arc and abs(here_radius - radius) <= 2.0 * band
            if near_entry or in_run_in:
                self._approach_committed = True
                self.get_logger().info(
                    f"On the collector circle {arc_to_home:.1f} m of arc short of the drop "
                    f"point; running in tangentially."
                )
            else:
                return entry

        lookahead = abs(float(self.get_parameter("approach_lookahead_m").value))
        # Never aim past the drop point: the run-in ends there, and a lookahead that
        # overshoots it would carry the rover on round the circle past its own pile.
        step = max(0.0, min(lookahead, arc_to_home))
        return on_circle(here_angle + step / radius)

    def distance_to_drop_band(self) -> float:
        """Metres still to travel before the drop band is entered; 0 once inside.

        The taper on the return leg has to be measured against THIS, not against
        the distance to the home point, and that mismatch is what was launching
        rocks out of the trailer. Arrival is decided by in_drop_band(), whose arc
        tolerance is 8 m, but the taper was measured to the home point over an 8 m
        slowdown -- so at the instant the band accepted the rover the taper was
        still commanding nearly full cruise, and drive_home stepped brake to 1.0
        (full in 0.1 s at brake_ramp_per_s=10). The rovers in the 3 h run parked
        6.6-7.8 m from home, i.e. right at the arc edge: those were emergency stops
        from ~4.5 m/s, and the load kept going.

        The rock approach never had this problem because it measures to the
        boundary it will actually stop at (see pickup_approach_target_speed) and so
        arrives at 0.03-0.46 m/s. Same idea here: the radial and arc errors are
        locally orthogonal, so the remaining travel is the hypotenuse of whatever
        each of them still has to give up.
        """
        errors = self.drop_band_errors()
        if errors is None:
            # Fall back to the straight-line distance rather than claiming arrival.
            if self.home is None or self.state is None:
                return 0.0
            return math.hypot(self.home[0] - self.state.x, self.home[1] - self.state.y)
        radial_err, arc_err, band, arc_tol = errors
        return math.hypot(max(0.0, radial_err - band), max(0.0, arc_err - arc_tol))

    def in_drop_band(self) -> Tuple[bool, str]:
        """True once the rover is anywhere in the drop band beside its builder.

        The drop point is not a surveyed spot -- the rocks only have to end up near
        this rank's builder. Insisting on a home_tolerance_m circle made rovers
        circle a point they were already parked next to: pure pursuit cannot
        converge on a target that sits inside its own turning radius, so it loops
        around it forever, and the rock never gets dropped.

        The accepted region is instead a ring segment hugging the collector circle:
        within drop_band_half_width_m of that radius (a band twice that wide,
        concentric with the circle, so the rover may stop short of or past it), and
        within drop_arc_tolerance_m of the drop point measured ALONG the circle. The
        arc limit is what keeps "near the builder" meaningful -- without it the whole
        ring would qualify, including the far side of the site.
        """
        errors = self.drop_band_errors()
        if errors is None:
            return (False, "band unavailable")
        radial_err, arc_err, band, arc_tol = errors
        if radial_err <= band and arc_err <= arc_tol:
            return (True, f"radial {radial_err:.2f}/{band:.2f} m, arc {arc_err:.2f}/{arc_tol:.2f} m")

        # STOP LINE. Once the rover has committed to the tangential run-in it is travelling
        # along the circle straight at the drop point, so "have I passed it" is a better
        # arrival test than "am I near it" -- and it is the one that cannot be missed. A
        # proximity test alone lets a rover that is carrying a little too much speed sail
        # through the tolerance in a single control period and then spend the rest of the
        # cycle turning back for a point behind it.
        if self._approach_committed and self.home is not None and self.state is not None:
            cx = float(self.get_parameter("site_center_x").value)
            cy = float(self.get_parameter("site_center_y").value)
            radius = math.hypot(self.home[0] - cx, self.home[1] - cy)
            if radius > 1e-6:
                home_angle = math.atan2(self.home[1] - cy, self.home[0] - cx)
                here_angle = math.atan2(self.state.y - cy, self.state.x - cx)
                arc_to_home = wrap_to_pi(home_angle - here_angle) * radius
                # Radial slack is looser here than the band, because having crossed the
                # line the rover is where it was going to be and stopping it is better
                # than sending it round again.
                if arc_to_home <= 0.0 and radial_err <= 2.0 * band:
                    return (
                        True,
                        f"passed the drop point by {-arc_to_home:.2f} m of arc, "
                        f"radial {radial_err:.2f}/{band:.2f} m",
                    )
        return (False, f"radial {radial_err:.2f}/{band:.2f} m, arc {arc_err:.2f}/{arc_tol:.2f} m")

    def drive_home(self) -> None:
        """Drive back to the spawn point, stop there, and report arrival."""
        if self.home is None:
            self.get_logger().warn(
                f"mission_done but no {self.home_pos_topic} received yet; holding.",
                once=True,
            )
            self.command = self.ramp_command(VehicleCommand(steering=0.0, throttle=0.0, brake=1.0))
            self.publish_command(self.command)
            return

        distance = math.hypot(self.home[0] - self.state.x, self.home[1] - self.state.y)
        tolerance = max(0.1, float(self.get_parameter("home_tolerance_m").value))
        in_band, band_why = self.in_drop_band()

        if self.parked_at_home or distance <= tolerance or in_band:
            # Speed at the instant the band accepted, i.e. the speed the full-brake
            # stop below has to absorb. This is the number that decides whether the
            # load stays in the bed, so it gets measured rather than assumed: it was
            # reaching ~4.5 m/s here before the taper was pointed at the band
            # boundary instead of the home point.
            if self._drop_entry_speed is None:
                self._drop_entry_speed = abs(self.state.speed)
            self.command = self.ramp_command(VehicleCommand(steering=0.0, throttle=0.0, brake=1.0))
            self.publish_command(self.command)
            self.update_stop_dwell()
            if not self.parked_at_home and self.is_fully_stopped():
                self.parked_at_home = True
                self.get_logger().info(
                    f"Parked at drop point ({distance:.2f} m from home, {band_why}, "
                    f"entered band at {self._drop_entry_speed:.3f} m/s); "
                    f"publishing {self.at_home_topic}=true."
                )
            # Keep asserting arrival: the dump sequencer may start after we do.
            if self.parked_at_home:
                self.at_home_pub.publish(Bool(data=True))
            return

        # Cruise home at the SAME speed as the outbound leg, then taper to the
        # BOUNDARY OF THE DROP BAND -- the place the rover will actually be told to
        # stop -- exactly as the rock approach tapers to the pickup boundary. See
        # distance_to_drop_band for why measuring to the home point instead was
        # throwing the load out of the trailer.
        cruise = max(0.0, float(self.get_parameter("target_speed_mps").value))
        final = max(0.0, float(self.get_parameter("home_approach_speed_mps").value))
        slowdown = max(1e-3, float(self.get_parameter("home_slowdown_distance_m").value))
        target_speed = final + (cruise - final) * clamp(self.distance_to_drop_band() / slowdown, 0.0, 1.0)

        # Steering first: it decides whether we are in the hard-turn regime, and a turn
        # needs forward speed to produce any yaw at all -- but it must not be taken at
        # cruise. Cornering acceleration is v^2 * curvature, and at lunar gravity the
        # traction guard only has mu*g ~ 1.3 m/s^2 to give: at 0.6 steering that caps the
        # corner at ~2.9 m/s, above which the guard cuts throttle and adds brake, and the
        # rover oscillates instead of turning. So the turn runs at its own fixed speed.
        # Not self.home: the return leg follows a two-waypoint path onto the collector
        # circle so the rover arrives running ALONG it. See home_approach_target.
        steering = self.compute_steering(self.home_approach_target())
        if self._hard_turning:
            target_speed = min(target_speed, float(self.get_parameter("reverse_turn_speed_mps").value))
        command = self.compute_speed_command(target_speed_override=target_speed)
        command.steering = steering
        self.command = self.ramp_command(command)
        self.publish_command(self.command)
        self.update_stop_dwell()

    def update_stop_dwell(self) -> None:
        """Accumulate time the chassis has been at/below the stop speed (reset if it moves)."""
        stop_speed = float(self.get_parameter("pickup_stop_speed_mps").value)
        if self.state is not None and abs(self.state.speed) <= stop_speed:
            self.stop_dwell_s += self.dt
        else:
            self.stop_dwell_s = 0.0

    def is_fully_stopped(self) -> bool:
        """True once the chassis has held the stop speed continuously for the dwell time."""
        return self.stop_dwell_s >= float(self.get_parameter("pickup_stop_dwell_s").value)

    def on_timer(self) -> None:
        if self.state is None:
            self.command = self.ramp_command(VehicleCommand())
            self.publish_command(self.command)
            return

        # The return leg outranks everything else, and does not need targetPos.
        if self.returning_home:
            self.drive_home()
            return

        # One deadline per target, checked in every state: it covers waiting on a
        # manipulator that never answers AND circling a rock whose approach angle never
        # comes good. Abandoning marks the rock completed so the rank moves on.
        if self._target_started_s is not None and 0 <= self.target_index < len(self.targets):
            elapsed = self.sim_now() - self._target_started_s
            deadline = float(self.get_parameter("target_done_timeout_sim_s").value)
            if deadline > 0.0 and elapsed >= deadline:
                self.get_logger().error(
                    f"targetPos[{self.target_index}]: stalled for {elapsed:.0f} s of sim "
                    f"(waiting={self.waiting_for_target_done}); abandoning this rock."
                )
                self.abandon_current_target()
                return

        if not self.targets:
            self.command = self.ramp_command(VehicleCommand())
            self.publish_command(self.command)
            return

        if self.is_straightening_after_done():
            self.command = self.ramp_command(VehicleCommand(steering=0.0, throttle=0.0, brake=1.0))
            self.publish_command(self.command)
            return

        if self.waiting_for_target_done:
            # Hold full brake and wait for a confirmed full stop before triggering the
            # grab. Send the pickup request exactly once per target; a timeout is a
            # safety net so a rover that can't perfectly settle still proceeds.
            self.command = self.ramp_command(VehicleCommand(steering=0.0, throttle=0.0, brake=1.0))
            self.publish_command(self.command)
            self.update_stop_dwell()

            if self.last_pickup_request_index != self.target_index:
                waited = self.sim_now() - (self.wait_start_s or self.sim_now())
                timed_out = waited >= float(self.get_parameter("pickup_stop_timeout_s").value)
                if self.is_fully_stopped() or timed_out:
                    self.publish_pickup_request_if_due(force=True)
                    self.get_logger().info(
                        f"targetPos[{self.target_index}] full stop "
                        f"(speed={self.state.speed:.3f} m/s, dwell={self.stop_dwell_s:.2f}s"
                        f"{', TIMEOUT' if timed_out and not self.is_fully_stopped() else ''}); "
                        f"sent pickup request."
                    )
            return

        switch_radius = max(0.0, float(self.get_parameter("switch_radius_m").value))
        while True:
            if not self.ensure_active_target():
                self.command = self.ramp_command(VehicleCommand())
                self.publish_command(self.command)
                return

            # Arrived AND correctly angled. A rover that is close but badly angled
            # keeps manoeuvring; the per-target deadline stops it circling forever.
            if not (self.is_target_within_switch_radius(switch_radius) and self.rock_is_in_pickup_angle_sector()):
                break

            rock_angle_deg = self.rock_angle_from_rear_reference_deg()
            self.get_logger().info(
                f"targetPos[{self.target_index}] is in pickup sector "
                f"(rear-reference angle={rock_angle_deg:.1f} deg); waiting for {self.target_done_topic}=true."
            )
            # Latch into the wait state and just brake. The pickup request is NOT sent
            # here -- it is deferred to the waiting-state handler above, which fires it
            # once only after a confirmed full stop. Reset the stop dwell / wait clock.
            self.waiting_for_target_done = True
            self.stop_dwell_s = 0.0
            self.wait_start_s = self.sim_now()
            self.command = self.ramp_command(VehicleCommand(steering=0.0, throttle=0.0, brake=1.0))
            self.publish_command(self.command)
            return

        target = self.get_drive_target()
        steering = self.compute_steering(target)
        speed_command = self.compute_speed_command(self.pickup_approach_target_speed(switch_radius))
        speed_command.steering = steering
        self.command = self.ramp_command(speed_command)
        self.publish_command(self.command)

    def compute_steering(self, target: Tuple[float, float]) -> float:
        target_x, target_y = target
        dx = target_x - self.state.x
        dy = target_y - self.state.y
        distance = math.hypot(dx, dy)
        alpha = wrap_to_pi(math.atan2(dy, dx) - self.state.yaw)
        wheelbase = max(1e-6, float(self.get_parameter("wheelbase_m").value))
        max_angle = max(1e-6, float(self.get_parameter("max_steering_angle_rad").value))

        # Pure pursuit's curvature, 2*sin(alpha)/L, is DEGENERATE when the target is
        # behind the vehicle: sin(pi) = 0, so a target dead astern commands ZERO
        # steering and the rover drives straight away from it forever. alpha = +-pi is
        # an unstable equilibrium, not a turn.
        #
        # This never showed on the way out -- rovers spawn facing outward with their
        # rocks ahead, so alpha stays small -- and appeared the first time a rover had
        # to come home, with the spawn point ~22 m astern. Outside the forward arc,
        # abandon pure pursuit and steer at full lock the short way round until the
        # target re-enters it. The hysteresis band keeps it from chattering on the
        # boundary once it does.
        turn_in_place = float(self.get_parameter("reverse_turn_alpha_rad").value)
        release = turn_in_place - float(self.get_parameter("reverse_turn_hysteresis_rad").value)
        if self._hard_turning:
            if abs(alpha) < max(0.0, release):
                self._hard_turning = False
        elif abs(alpha) > turn_in_place:
            self._hard_turning = True
        if self._hard_turning:
            # copysign, not a curvature: at alpha = +-pi the sign is arbitrary, so pick
            # one and commit to it rather than dithering between equal-length turns.
            # Bounded, NOT full lock -- see reverse_turn_steering above.
            turn_steer = clamp(float(self.get_parameter("reverse_turn_steering").value), 0.0, 1.0)
            return math.copysign(turn_steer, alpha if alpha != 0.0 else 1.0)

        # Bound the lookahead. Using the raw distance to a far target flattens the
        # curvature to nothing (2*sin(alpha)/22 m barely steers), which is the second
        # reason the return leg tracked so poorly. Pure pursuit wants a lookahead set by
        # the vehicle, not by how far away the goal happens to be.
        lookahead = clamp(
            distance,
            float(self.get_parameter("lookahead_min_m").value),
            max(1e-3, float(self.get_parameter("lookahead_max_m").value)),
        )
        curvature = 2.0 * math.sin(alpha) / lookahead
        steering_angle = math.atan(wheelbase * curvature)
        return clamp(steering_angle / max_angle, -1.0, 1.0)

    def ensure_active_target(self) -> bool:
        if 0 <= self.target_index < len(self.targets) and self.target_index not in self.completed_targets:
            return True

        remaining = [i for i in range(len(self.targets)) if i not in self.completed_targets]
        if not remaining:
            return False

        self.target_index = min(
            remaining,
            key=lambda i: math.hypot(self.targets[i][0] - self.state.x, self.targets[i][1] - self.state.y),
        )
        self.drive_target_index = -1
        self._target_started_s = self.sim_now()
        target_x, target_y, _target_z = self.targets[self.target_index]
        distance = math.hypot(target_x - self.state.x, target_y - self.state.y)
        self.get_logger().info(
            f"Selected nearest targetPos[{self.target_index}] at ({target_x:.2f}, {target_y:.2f}), "
            f"distance={distance:.2f} m."
        )
        return True

    def get_drive_target(self) -> Tuple[float, float]:
        rock_x, rock_y, _rock_z = self.targets[self.target_index]
        if self.drive_target_index != self.target_index:
            self.drive_target_offset = self.compute_rock_side_offset(rock_x, rock_y)
            self.drive_target_index = self.target_index
            target_x = rock_x + self.drive_target_offset[0]
            target_y = rock_y + self.drive_target_offset[1]
            self.get_logger().info(
                f"targetPos[{self.target_index}] rock=({rock_x:.2f}, {rock_y:.2f}) "
                f"drive_target=({target_x:.2f}, {target_y:.2f})"
            )

        return rock_x + self.drive_target_offset[0], rock_y + self.drive_target_offset[1]

    def compute_rock_side_offset(self, rock_x: float, rock_y: float) -> Tuple[float, float]:
        offset = max(0.0, float(self.get_parameter("rock_side_offset_m").value))
        to_rock_x = rock_x - self.state.x
        to_rock_y = rock_y - self.state.y
        distance = math.hypot(to_rock_x, to_rock_y)
        if distance < 1e-6:
            approach_x = math.cos(self.state.yaw)
            approach_y = math.sin(self.state.yaw)
        else:
            approach_x = to_rock_x / distance
            approach_y = to_rock_y / distance

        left_x = -approach_y
        left_y = approach_x
        left_offset = (offset * left_x, offset * left_y)
        right_offset = (-offset * left_x, -offset * left_y)

        def heading_error_for(candidate_offset: Tuple[float, float]) -> float:
            target_x = rock_x + candidate_offset[0]
            target_y = rock_y + candidate_offset[1]
            return abs(wrap_to_pi(math.atan2(target_y - self.state.y, target_x - self.state.x) - self.state.yaw))

        if heading_error_for(left_offset) <= heading_error_for(right_offset):
            return left_offset
        return right_offset

    def compute_speed_command(self, target_speed_override: Optional[float] = None) -> VehicleCommand:
        if target_speed_override is None:
            target_speed = max(0.0, float(self.get_parameter("target_speed_mps").value))
        else:
            target_speed = max(0.0, target_speed_override)

        if target_speed >= self.state.speed and self.ramped_target_speed < self.state.speed:
            self.ramped_target_speed = self.state.speed

        target_ramp_mps2 = max(0.0, float(self.get_parameter("target_speed_ramp_mps2").value))
        if target_speed < self.ramped_target_speed:
            target_ramp_mps2 = max(target_ramp_mps2, target_speed / max(self.dt, 1e-6))

        target_ramp = target_ramp_mps2 * self.dt
        self.ramped_target_speed = approach(self.ramped_target_speed, target_speed, target_ramp)

        speed_error = self.ramped_target_speed - self.state.speed
        tolerance = max(0.0, float(self.get_parameter("speed_tolerance_mps").value))
        if abs(speed_error) <= tolerance:
            return VehicleCommand(steering=0.0, throttle=0.0, brake=0.0)

        effort = max(0.0, float(self.get_parameter("speed_kp").value)) * speed_error
        throttle = clamp(effort, 0.0, 1.0)
        brake = clamp(-effort, 0.0, 1.0)

        # BREAKAWAY: a stopped rover needs full throttle to start moving again.
        #
        # Below 1 m/s the TMeasy tire stops using its slip curve and switches to a
        # Dahl bristle friction model, which Chrono documents as acting "like a
        # spring which enables holding of a vehicle on a slope without creeping".
        # It is a STICTION model: to get moving, thrust must beat the full Coulomb
        # limit mu*Fz, not merely the rolling resistance.
        #
        # Measured at a real stall: mu*Fz = 0.75 * 2023 N = 1517 N to break out.
        # The engine map gives 37 Nm at 0 rpm, which through the driveline is
        # 1696 N at FULL throttle but only 1196 N at the 0.705 this proportional
        # controller was asking for. 1196 < 1517, so the rover stayed put -- and
        # because it stayed put, speed_error never grew, so throttle never rose.
        # A closed deadlock: stopped forever on flat ground with the engine idling
        # at 1.6 rpm, every wheel loaded and gripping.
        #
        # Saturating the throttle once the rover has demonstrably failed to move
        # for stall_dwell gives 1696 N > 1517 N and breaks the bristles. The dwell
        # keeps this out of normal launches, which accelerate away long before it.
        # Engage and release on DIFFERENT speeds, or this becomes a limit cycle.
        # Releasing the moment speed crept back over stall_speed dropped throttle
        # straight back to ~0.6, which is under the stiction threshold again, so the
        # rover stuck, broke free, stuck again -- crawling a pickup approach at
        # 0.06 m/s with 70+ breakaways instead of driving it. Hold full throttle
        # until it is properly rolling. Never force it past what was actually
        # commanded, though: the pickup approach deliberately targets a near-stop so
        # the arm can solve IK against a stationary base.
        stall_speed = 0.08
        stall_dwell = 1.0
        release_speed = min(0.5, max(2.0 * stall_speed, self.ramped_target_speed))

        if self._breakaway_active:
            if abs(self.state.speed) >= release_speed or brake > 0.0:
                self._breakaway_active = False
                self._stalled_since = None
            else:
                throttle = 1.0
                brake = 0.0
        elif self.ramped_target_speed > 0.1 and abs(self.state.speed) < stall_speed and brake <= 0.0:
            if self._stalled_since is None:
                self._stalled_since = self.now_seconds()
            elif self.now_seconds() - self._stalled_since >= stall_dwell:
                self._breakaway_active = True
                self.get_logger().warn(
                    f"Not moving ({abs(self.state.speed):.4f} m/s) with throttle {throttle:.2f} "
                    f"for {stall_dwell:.1f} s -- tire stiction needs full throttle to break; "
                    f"holding full throttle until {release_speed:.2f} m/s."
                )
                throttle = 1.0
                brake = 0.0
        else:
            self._stalled_since = None

        return VehicleCommand(steering=0.0, throttle=throttle, brake=brake)

    def ramp_command(self, target: VehicleCommand) -> VehicleCommand:
        throttle_delta = max(0.0, float(self.get_parameter("throttle_ramp_per_s").value)) * self.dt
        brake_delta = max(0.0, float(self.get_parameter("brake_ramp_per_s").value)) * self.dt
        steering_delta = max(0.0, float(self.get_parameter("steering_ramp_per_s").value)) * self.dt

        return VehicleCommand(
            steering=approach(self.command.steering, target.steering, steering_delta),
            throttle=approach(self.command.throttle, target.throttle, throttle_delta),
            brake=approach(self.command.brake, target.brake, brake_delta),
        )

    def publish_command(self, command: VehicleCommand) -> None:
        msg = Float64MultiArray()
        msg.data = [
            clamp(command.steering, -1.0, 1.0),
            clamp(command.throttle, 0.0, 1.0),
            clamp(command.brake, 0.0, 1.0),
        ]
        self.command_pub.publish(msg)

    def publish_pickup_request_if_due(self, force: bool = False) -> None:
        if not (0 <= self.target_index < len(self.targets)):
            return

        now = self.now_seconds()
        request_rate = max(1e-6, float(self.get_parameter("pickup_request_rate_hz").value))
        request_period = 1.0 / request_rate
        if (
            not force
            and self.last_pickup_request_index == self.target_index
            and self.last_pickup_request_time_s is not None
            and now - self.last_pickup_request_time_s < request_period
        ):
            return

        rock_x, rock_y, rock_z = self.targets[self.target_index]
        msg = Float64MultiArray()
        msg.data = [float(self.target_index), rock_x, rock_y, rock_z]
        self.pickup_request_pub.publish(msg)
        self.last_pickup_request_time_s = now
        self.last_pickup_request_index = self.target_index

    def rear_reference_position(self) -> Tuple[float, float]:
        offset = max(0.0, float(self.get_parameter("rear_reference_offset_m").value))
        return (
            self.state.x - offset * math.cos(self.state.yaw),
            self.state.y - offset * math.sin(self.state.yaw),
        )

    def rock_angle_from_rear_reference_deg(self) -> float:
        rock_x, rock_y, _rock_z = self.targets[self.target_index]
        ref_x, ref_y = self.rear_reference_position()
        return math.degrees(wrap_to_pi(math.atan2(rock_y - ref_y, rock_x - ref_x) - self.state.yaw))

    def rock_is_in_pickup_angle_sector(self) -> bool:
        angle = self.rock_angle_from_rear_reference_deg()
        angle_min = abs(float(self.get_parameter("pickup_angle_min_deg").value))
        angle_max = abs(float(self.get_parameter("pickup_angle_max_deg").value))
        if angle_min > angle_max:
            angle_min, angle_max = angle_max, angle_min

        return angle_min <= abs(angle) <= angle_max

    def is_target_within_switch_radius(self, switch_radius: float) -> bool:
        target_x, target_y = self.get_drive_target()
        ref_x, ref_y = self.rear_reference_position()
        return math.hypot(target_x - ref_x, target_y - ref_y) <= switch_radius

    def is_target_in_pickup_position(self, switch_radius: float) -> bool:
        return self.is_target_within_switch_radius(switch_radius) and self.rock_is_in_pickup_angle_sector()

    def pickup_approach_target_speed(self, switch_radius: float) -> float:
        target_speed = max(0.0, float(self.get_parameter("target_speed_mps").value))
        target_x, target_y = self.get_drive_target()
        ref_x, ref_y = self.rear_reference_position()
        distance_to_boundary = max(0.0, math.hypot(target_x - ref_x, target_y - ref_y) - switch_radius)

        boundary_speed = max(0.0, float(self.get_parameter("pickup_boundary_speed_mps").value))
        if target_speed <= 0.0:
            return 0.0

        slowdown_offset = max(1e-6, float(self.get_parameter("pickup_slowdown_offset_m").value))
        speed_scale = clamp(distance_to_boundary / slowdown_offset, 0.0, 1.0)
        approach_speed = boundary_speed + speed_scale * (target_speed - boundary_speed)

        min_approach_speed = max(0.0, float(self.get_parameter("pickup_min_approach_speed_mps").value))
        return min(target_speed, max(min_approach_speed, approach_speed))

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def sim_now(self) -> float:
        """Simulation seconds, for deadlines on things the ROVER has to do.

        Use this, not now_seconds(), whenever the timeout is about physical
        progress -- settling to a stop, manoeuvring into position. The wall clock
        runs ~20x faster than this sim and the ratio is not fixed, so a wall-clock
        allowance silently shrinks to a twentieth of its stated value. Falls back
        to wall time only if the sim is too old to publish a clock.
        """
        return self.sim_time if self.sim_time is not None else self.now_seconds()

    def is_straightening_after_done(self) -> bool:
        if self.straighten_until_time_s is None:
            return False

        if self.sim_now() < self.straighten_until_time_s:
            return True

        self.straighten_until_time_s = None
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
