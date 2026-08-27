import math
import time
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
        self._last_speed_trace = None
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("target_speed_mps", 1.0)
        # Speed loop gain. Raised from 0.55, where the achieved speed was a function of the
        # TARGET rather than of what the rover could do.
        #
        # It is a bare proportional loop, so it settles wherever throttle balances drag and
        # keeps whatever steady-state error that implies. At 0.55 an error of 1.49 m/s asks
        # for 0.82 throttle -- not saturated -- so the rover sat at 1.48 m/s while being
        # commanded 2.97, measured over 778 trace samples with no cap binding on any of
        # them: hard_turning=0 throughout and the traction bound at 9-40 m/s. Lowering the
        # launch target from 5.0 to 3.0 therefore cut the throttle rather than the speed
        # limit, which is why top speed fell from 2.46-2.61 m/s to 1.57-1.86 m/s.
        #
        # 1.2 saturates the throttle until the rover is within 0.83 m/s of target and tapers
        # inside that. Cornering stays bounded by traction_speed_limit, which is computed
        # from the ramped steering and holds full lock to 2.2 m/s, and the kingpin stops
        # backstop the knuckle if any of that is wrong.
        self.declare_parameter("speed_kp", 1.2)
        self.declare_parameter("speed_tolerance_mps", 0.08)
        self.declare_parameter("target_speed_ramp_mps2", 10.0)
        self.declare_parameter("throttle_ramp_per_s", 10.0)
        self.declare_parameter("brake_ramp_per_s", 10.0)
        # Steering slew-rate limit (units/s of the [-1,1] command). Caps how fast the
        # commanded steer angle can change so pure-pursuit corrections come in smoothly
        # instead of snapping. 2.5 => full lock in ~0.4 s. Lower = gentler/slower.
        # 1.5 => full lock in ~0.7 s. Was 2.5 (~0.4 s), which slams the lateral load into
        # the axle rather than building it; the jerk is what spikes the joint, not the
        # steady-state cornering force.
        self.declare_parameter("steering_ramp_per_s", 1.5)
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
        # 0.40 of full lock, not 0.60. At wheelbase_m 2.5 that is atan-free arithmetic:
        # 0.40 * max_steering_angle_rad = 0.24 rad, so the arc radius is
        # 2.5 / tan(0.24) = 10.2 m against 6.6 m at 0.60. On SCM the tires do not slip
        # cleanly at the limit, they BULLDOZE -- soil piles against the sidewall and the
        # lateral load goes into the axle instead of into yaw -- and the recovery from that
        # is a locked steering axle that ruins the rest of the run. The return leg has 20+
        # m of open ground, so the wider arc costs nothing but a couple of seconds.
        self.declare_parameter("reverse_turn_steering", 0.40)
        self.declare_parameter("reverse_turn_speed_mps", 1.0)
        # Lateral acceleration the regolith will actually give, m/s^2. Cornering demand is
        # v^2 * curvature. mu*g at lunar gravity is 0.9 * 1.62 = 1.46, and SCM bulldozing
        # resistance is not a clean friction circle, so this sits just under it.
        #
        # Was 1.0, which combined with feeding the bound the raw steering DEMAND cost more
        # than half the rover's speed for a load it never generated. 1.3 leaves the bound
        # inactive at cruise -- at the 0.56 of lock the uprights actually reached, it
        # allows 3.05 m/s against a 3.0 m/s target -- and still holds full lock to 2.2 m/s,
        # which is the case it exists for.
        self.declare_parameter("max_lateral_accel_mps2", 1.3)
        # Wall seconds between speed-limiter trace lines; 0 disables. See speed_trace.
        # 10 s rather than 1: an unattended 3 h 40 m cluster slot at 1 s would put 13200
        # lines per rover into the job log, and with four robots that is the log.
        self.declare_parameter("speed_trace_period_s", 10.0)
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
        # Instead it runs in on an arc that lies OUTSIDE the collector circle and touches
        # it only at the drop point, arriving clockwise while the builder walks
        # counter-clockwise. See approach_arc_radius_m below for the construction and for
        # why the previous ring-following version drove into builders.
        #
        # Reaching the entry waypoint is not required to be precise: approach_capture_m
        # accepts it generously, and everything after that is arc-following.
        self.declare_parameter("approach_capture_m", 5.0)
        self.declare_parameter("approach_lookahead_m", 6.0)
        # Radius of the run-in arc, and the whole reason this file was rewritten.
        #
        # The old run-in put its entry waypoint ON the collector circle, arc_m CLOCKWISE
        # of the drop point, and then followed the circle in. Clockwise is where the
        # BUILDER is -- it walks counter-clockwise towards the drop point -- so the last
        # 12 m of every return leg was driven along the builder's own lane with 4 m of
        # radius between them (37 vs 33, hull half-width 1.343). Collectors hit builders.
        #
        # Instead the run-in is an arc that lies OUTSIDE the collector circle and touches
        # it only at the drop point. Put its centre on the drop point's own ray at radius
        # (ring + rho): then the closest that circle ever comes to the site centre is
        # exactly `ring`, at the drop point, and every other point on it is further out.
        # So the rover can never be swept inboard onto the builder, and its heading at the
        # touch point is tangential to the ring by construction.
        #
        # rho is the only knob. It must exceed the rover's ~5 m turn radius with margin,
        # because this curve is driven under load on lunar traction; 12 m is 2.4x that,
        # and at home_approach_speed_mps=1.0 a quarter arc is ~19 s of sim. Larger is
        # smoother and slower; below ~8 m the curve starts fighting the trailer.
        self.declare_parameter("approach_arc_radius_m", 12.0)
        # How far back along the arc the rover aims before it has captured the arc. A
        # quarter turn puts the entry point well outside the ring (at rho=12, ring=37 that
        # is r=50.4) and on the COUNTER-CLOCKWISE side of the drop point -- the side the
        # builder is never on.
        self.declare_parameter("approach_entry_sweep_rad", math.pi / 2.0)
        # Radial tolerance for deciding the rover is on the arc.
        self.declare_parameter("approach_arc_capture_m", 4.0)
        # Builder feedback. The drop point is placed this many wall slots AHEAD of the
        # slot the builder is currently consuming, so the load lands where it will next
        # need rocks rather than where the harvest cycle counter happened to point.
        #
        # 1 slot, not more, and the arm is why: arm base sits at r=33.09, the pile at
        # r=37, and one slot is 0.03 rad, so the base-to-pile distance runs 3.91 m at 0
        # slots, 4.04 at 1, 4.43 at 2, and 5.01 at 3 -- against feedstock_reach_max=5.0 in
        # BuilderArmRosBridge. Three slots ahead is already refused. Reach, not collision,
        # is what bounds this now that the run-in no longer crosses the builder's lane.
        self.declare_parameter("drop_slots_ahead", 1.0)
        # Must match wall_slot_pitch_rad in src/RobotLayout.h: 0.5 m of course at r=30,
        # so 0.5/30. There is no way to read the C++ constant from here, so it is mirrored
        # by hand -- change one and you must change the other, or the collector aims its
        # drop at a slot the builder is not working.
        self.declare_parameter("slot_pitch_rad", 0.0166667)
        # Which builder feeds this collector. Each rank owns one of each, so 0 means "the
        # one with my own id" -- resolved after robot_id is read, below.
        self.declare_parameter("builder_id", 0)
        # Drop back to the sim's own drop point if the builder has said nothing for this
        # long (wall seconds). Losing the topic must not strand a loaded rover.
        self.declare_parameter("builder_status_timeout_s", 5.0)
        # WHICH POINT ON THE RIG has to be in the drop band. The load leaves from the bed,
        # not from the tractor, so accepting arrival at the tractor's own reference parks
        # the trailer short of the drop point -- and on a clockwise run-in "short" is the
        # counter-clockwise side, i.e. AWAY from the builder, so the rocks tumble away from
        # the machine that has to pick them up. The sim publishes the trailer's rear axle
        # on /robot_N/trailer_state; this offset is only the fallback for when it has not
        # arrived yet, measured back along the tractor heading.
        self.declare_parameter("drop_reference_offset_m", 3.5)
        self.declare_parameter("trailer_state_timeout_s", 5.0)
        # ACCEPT ON REACH, not on radial/arc proxies.
        #
        # The builder only cares about one number: how far the pile ends up from its arm
        # base. BuilderArmRosBridge refuses anything outside [2.0, 5.0] m of it. Radial and
        # arc error do not cost the same against that budget -- a metre of outward radial
        # error is a metre of reach, a metre of arc is about 0.4 m -- so no single radial
        # tolerance is both safe and permissive. Measured: parking on the old tractor-based
        # test put the axle 1.9 m radially and 2.5 m of arc off the drop point, i.e. ~5.8 m
        # from the arm base, and all four builders sat starved with "none within 2.0-5.0 m".
        #
        # So the drop is accepted when the trailer axle is inside a MARGIN window on the
        # real distance to /builder_N/arm_base_pose. The window is inset from the hard
        # limits because the load still has to leave a 55 deg bed and roll to rest before
        # it freezes (dumped_rock_settle_* in RobotRig.cpp): the pile lands near the axle,
        # not exactly on it.
        # WHICH point on the rig is judged. "gate" is the rear gate edge centre -- the lip
        # the load pours over, and the point the bed is hinged on so it stays fixed in
        # space while the tub rises. "axle" is the rear axle, roughly a metre forward of
        # it, which drops the load about a metre short: some rocks land inside the
        # builder's pickup radius and some outside. "offset" forces the heading-projected
        # fallback. Judging at the gate is what makes the rig carry on those last few
        # metres before it tips.
        self.declare_parameter("drop_reference_point", "gate")
        # REPORTING ONLY, both of these and the reach window below. How tangential the
        # trailer was and how far the pile ended up from the arm base are worth knowing --
        # they are the numbers that say whether the run-in geometry is right -- but neither
        # may gate the drop: see drive_home for the orbit that gating caused. Measured from
        # the trailer's own axis (gate -> axle), not the tractor's. 0 omits the angle.
        self.declare_parameter("drop_heading_tolerance_deg", 20.0)
        self.declare_parameter("drop_reach_min_m", 2.6)
        self.declare_parameter("drop_reach_max_m", 4.5)
        # Radial nudge on the drop point, inboard negative. The ring radius comes from the
        # sim, and nominal reach at 37 m is 3.94 m against a 5.0 m ceiling -- only ~1 m of
        # headroom for outward error, against ~1.9 m of floor before the 2.0 m minimum. Set
        # this to about -1.0 to buy back a metre of the tighter side if acceptance tuning is
        # not enough on its own.
        self.declare_parameter("drop_radius_offset_m", 0.0)
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
        builder_id = int(self.get_parameter("builder_id").value)
        self.builder_id = builder_id if builder_id > 0 else self.robot_id
        self.builder_status_topic = f"/builder_{self.builder_id}/arm_status"
        # Latest [slot, usable_rocks, slot_angle_rad] from the builder, and the wall clock
        # it arrived on. None until the first message: until then the drop point is the
        # sim's own, which is the pre-existing behaviour.
        self.builder_slot: Optional[int] = None
        self.builder_rocks_left: Optional[int] = None
        self.builder_slot_angle: Optional[float] = None
        self.builder_status_stamp: Optional[float] = None
        self._logged_builder_drop = False
        self.trailer_state_topic = f"/robot_{self.robot_id}/trailer_state"
        self.trailer_axle: Optional[Tuple[float, float]] = None
        self.trailer_axle_stamp: Optional[float] = None
        self.trailer_gate: Optional[Tuple[float, float]] = None
        self.builder_base_topic = f"/builder_{self.builder_id}/arm_base_pose"
        self.builder_arm_base: Optional[Tuple[float, float]] = None
        self.builder_arm_base_stamp: Optional[float] = None
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
        self.create_subscription(
            Float64MultiArray, self.builder_status_topic, self.on_builder_status, 10
        )
        self.create_subscription(
            Float64MultiArray, self.trailer_state_topic, self.on_trailer_state, 10
        )
        self.create_subscription(
            Float64MultiArray, self.builder_base_topic, self.on_builder_arm_base, 10
        )

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

    def on_builder_status(self, msg: Float64MultiArray) -> None:
        """Track where the builder is consuming and how much it has left.

        Indices 6-8 are appended fields (see BuilderArmRosBridge.h); a shorter message is
        an older sim, and is ignored rather than misread.
        """
        if len(msg.data) < 9:
            return
        self.builder_slot = int(msg.data[6])
        self.builder_rocks_left = int(msg.data[7])
        self.builder_slot_angle = float(msg.data[8])
        self.builder_status_stamp = time.monotonic()

    def on_trailer_state(self, msg: Float64MultiArray) -> None:
        """Track the trailer's rear axle -- the point the load actually leaves from.

        Indices 3-5 are [valid, x, y], appended (see RosTrailerBridge.h). The valid flag
        exists so (0, 0) is never mistaken for the site origin.
        """
        if len(msg.data) < 6 or msg.data[3] < 0.5:
            return
        self.trailer_axle = (float(msg.data[4]), float(msg.data[5]))
        self.trailer_axle_stamp = time.monotonic()
        # Indices 6-8 are [valid, x, y] for the rear gate edge centre, appended after the
        # axle. Absent on an older sim, in which case the axle is still used.
        if len(msg.data) >= 9 and msg.data[6] >= 0.5:
            self.trailer_gate = (float(msg.data[7]), float(msg.data[8]))

    def on_builder_arm_base(self, msg: Float64MultiArray) -> None:
        """Track the builder's IK frame origin -- the point its reach test measures from."""
        if len(msg.data) < 2:
            return
        self.builder_arm_base = (float(msg.data[0]), float(msg.data[1]))
        self.builder_arm_base_stamp = time.monotonic()

    def reach_to_builder(self) -> Optional[float]:
        """Distance from the trailer axle to the builder's arm base, or None.

        This is the quantity BuilderArmRosBridge tests against [2.0, 5.0] m before it will
        offer a rock to the arm, so it is the only honest measure of whether a drop here
        can be picked up.
        """
        timeout = abs(float(self.get_parameter("builder_status_timeout_s").value))
        if self.builder_arm_base is None or self.builder_arm_base_stamp is None:
            return None
        if timeout > 0.0 and (time.monotonic() - self.builder_arm_base_stamp) > timeout:
            return None
        ref_x, ref_y = self.drop_reference()
        return math.hypot(ref_x - self.builder_arm_base[0], ref_y - self.builder_arm_base[1])

    def trailer_points(self) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """(axle, gate) if both are fresh, else None. Their difference is the trailer axis."""
        timeout = abs(float(self.get_parameter("trailer_state_timeout_s").value))
        if self.trailer_axle is None or self.trailer_gate is None or self.trailer_axle_stamp is None:
            return None
        if timeout > 0.0 and (time.monotonic() - self.trailer_axle_stamp) > timeout:
            return None
        return (self.trailer_axle, self.trailer_gate)

    def trailer_heading(self) -> Optional[float]:
        """Heading of the trailer itself, from the gate forward to the axle.

        The tractor's yaw is not a substitute: the rig is articulated, and through the
        run-in curve the trailer lags the tractor by a few degrees -- which is precisely
        the error that turns a line of rock along the ring into a heap across it.
        """
        pts = self.trailer_points()
        if pts is None:
            return None
        (ax, ay), (gx, gy) = pts
        dx, dy = ax - gx, ay - gy
        if math.hypot(dx, dy) < 1e-6:
            return None
        return math.atan2(dy, dx)

    def drop_reference(self) -> Tuple[float, float]:
        """The point on the rig that has to be inside the drop band.

        Preference order, and why: the REAR GATE EDGE CENTRE, because that is the lip the
        load pours over and the point the bed is hinged on so it stays fixed while the tub
        rises; then the rear axle, about a metre forward of it; then the tractor pose
        pushed back along its own heading, which is only right on a straight run and wrong
        by the articulation angle on a curve.
        """
        want = str(self.get_parameter("drop_reference_point").value).strip().lower()
        timeout = abs(float(self.get_parameter("trailer_state_timeout_s").value))
        fresh = self.trailer_axle_stamp is not None and (
            timeout <= 0.0 or (time.monotonic() - self.trailer_axle_stamp) <= timeout
        )
        if fresh and want != "offset":
            if want == "gate" and self.trailer_gate is not None:
                return self.trailer_gate
            if self.trailer_axle is not None:
                return self.trailer_axle
        offset = max(0.0, float(self.get_parameter("drop_reference_offset_m").value))
        return (
            self.state.x - offset * math.cos(self.state.yaw),
            self.state.y - offset * math.sin(self.state.yaw),
        )

    def drop_point(self) -> Optional[Tuple[float, float]]:
        """Where to put the load: the builder's next slot, at the sim's ring radius.

        The sim publishes a drop point on /robot_N/homePos derived from the harvest cycle
        counter, which says nothing about how far the builder actually got. This keeps that
        point's RADIUS -- the ring geometry and the arm's reach envelope were sized
        together, so the radius is not ours to move -- and overrides only its ANGLE, to
        drop_slots_ahead slots past the slot the builder is working now.

        Falls back to the sim's point when the builder has not reported, or has gone quiet,
        so a lost topic degrades to the old behaviour instead of stranding a loaded rover.
        """
        if self.home is None:
            return None
        if self.builder_slot_angle is None or self.builder_status_stamp is None:
            return self.home
        timeout = abs(float(self.get_parameter("builder_status_timeout_s").value))
        if timeout > 0.0 and (time.monotonic() - self.builder_status_stamp) > timeout:
            return self.home

        cx = float(self.get_parameter("site_center_x").value)
        cy = float(self.get_parameter("site_center_y").value)
        radius = math.hypot(self.home[0] - cx, self.home[1] - cy)
        if radius < 1e-6:
            return self.home
        pitch = float(self.get_parameter("slot_pitch_rad").value)
        ahead = float(self.get_parameter("drop_slots_ahead").value)
        angle = self.builder_slot_angle + ahead * pitch
        # Inboard nudge trades outward-error headroom for inward: see drop_radius_offset_m.
        radius = max(1.0, radius + float(self.get_parameter("drop_radius_offset_m").value))
        target = (cx + radius * math.cos(angle), cy + radius * math.sin(angle))
        if not self._logged_builder_drop:
            self._logged_builder_drop = True
            self.get_logger().info(
                f"drop point follows builder {self.builder_id}: slot {self.builder_slot}, "
                f"{self.builder_rocks_left} rock(s) unlaid, dropping {ahead:.0f} slot(s) "
                f"ahead at ({target[0]:.2f}, {target[1]:.2f})."
            )
        return target

    def drop_band_errors(self) -> Optional[Tuple[float, float, float, float]]:
        """(radial_err, arc_err, band_half_width, arc_tolerance) for the drop band.

        Returns None when the band cannot be evaluated. Shared by in_drop_band (is
        the rover there yet) and distance_to_drop_band (how much further), so the
        arrival test and the speed taper can never disagree about where the band is.
        """
        drop = self.drop_point()
        if drop is None or self.state is None:
            return None
        cx = float(self.get_parameter("site_center_x").value)
        cy = float(self.get_parameter("site_center_y").value)
        band = abs(float(self.get_parameter("drop_band_half_width_m").value))
        arc_tol = abs(float(self.get_parameter("drop_arc_tolerance_m").value))
        if band <= 0.0 or arc_tol <= 0.0:
            return None

        home_radius = math.hypot(drop[0] - cx, drop[1] - cy)
        if home_radius < 1e-6:
            return None
        # The REAR AXLE against the band, not the tractor: see drop_reference().
        ref_x, ref_y = self.drop_reference()
        here_radius = math.hypot(ref_x - cx, ref_y - cy)
        radial_err = abs(here_radius - home_radius)

        home_angle = math.atan2(drop[1] - cy, drop[0] - cx)
        here_angle = math.atan2(ref_y - cy, ref_x - cx)
        arc_err = abs(wrap_to_pi(here_angle - home_angle)) * home_radius
        return (radial_err, arc_err, band, arc_tol)

    def home_approach_target(self) -> Tuple[float, float]:
        """Where to steer on the return leg, so the rover arrives ALONG the ring.

        The run-in is a circle of radius rho whose centre sits on the drop point's own ray
        at radius (ring + rho). That circle touches the collector ring at exactly one
        point -- the drop point -- and lies strictly outside it everywhere else, because
        its closest approach to the site centre is (ring + rho) - rho = ring. Two things
        follow for free:

          * the rover is never carried inboard of the ring, so it cannot be swept into the
            builder orbiting 4 m inside it;
          * the arc is tangent to the ring at the drop point, so the heading there is
            tangential and the rear-discharging trailer pours along the circumference.

        Travel is CLOCKWISE about the site, which is counter-clockwise about the arc's own
        centre. That is the direction that matters: the builder walks counter-clockwise
        TOWARDS the drop point, so it is always on the clockwise side, and a clockwise
        arrival descends from the other side entirely. It also points the trailer
        counter-clockwise, laying the load ahead of the builder rather than behind it.

        Two stages, latched by _approach_committed so the switch cannot chatter:

          not committed -- steer at an entry point entry_sweep radians back along the arc.
                           Commit once within approach_capture_m of it, or once the rover
                           is already on the arc.
          committed     -- steer at a point ON the arc, approach_lookahead_m of arc ahead
                           of the rover's own position around it, never past the drop
                           point. Unlike the old ring-following version the target keeps
                           sliding all the way in, so the final metres are still
                           path-following rather than a straight run at a fixed point.

        Falls back to the drop point itself if the geometry cannot be evaluated, so a
        missing parameter degrades rather than strands.
        """
        drop = self.drop_point()
        if drop is None or self.state is None:
            if drop is not None:
                return drop
            return (self.state.x, self.state.y) if self.state is not None else (0.0, 0.0)

        cx = float(self.get_parameter("site_center_x").value)
        cy = float(self.get_parameter("site_center_y").value)
        ring = math.hypot(drop[0] - cx, drop[1] - cy)
        rho = abs(float(self.get_parameter("approach_arc_radius_m").value))
        if ring < 1e-6 or rho < 1e-6:
            return drop

        drop_angle = math.atan2(drop[1] - cy, drop[0] - cx)
        # Arc centre: outward along the drop point's ray, one rho past the ring.
        ax = cx + (ring + rho) * math.cos(drop_angle)
        ay = cy + (ring + rho) * math.sin(drop_angle)

        def on_arc(phi: float) -> Tuple[float, float]:
            return (ax + rho * math.cos(phi), ay + rho * math.sin(phi))

        # Angles measured about the ARC centre. The drop point lies on the far side of the
        # arc from the site centre, i.e. at drop_angle + pi as seen from the arc centre.
        phi_drop = wrap_to_pi(drop_angle + math.pi)
        phi_here = math.atan2(self.state.y - ay, self.state.x - ax)
        # Travel about the arc centre is counter-clockwise (= clockwise about the site), so
        # remaining sweep is measured in the positive direction and is always in [0, 2pi).
        sweep_to_drop = (phi_drop - phi_here) % (2.0 * math.pi)
        arc_radius_err = abs(math.hypot(self.state.x - ax, self.state.y - ay) - rho)

        if not self._approach_committed:
            entry_sweep = abs(float(self.get_parameter("approach_entry_sweep_rad").value))
            entry = on_arc(phi_drop - entry_sweep)
            capture = abs(float(self.get_parameter("approach_capture_m").value))
            arc_capture = abs(float(self.get_parameter("approach_arc_capture_m").value))
            near_entry = math.hypot(entry[0] - self.state.x, entry[1] - self.state.y) <= capture
            # Already on the arc, somewhere in the run-in sweep: driving back to a waypoint
            # the rover has effectively reached would only cost it the heading it has.
            on_run_in = arc_radius_err <= arc_capture and sweep_to_drop <= entry_sweep
            if near_entry or on_run_in:
                self._approach_committed = True
                self.get_logger().info(
                    f"Captured the run-in arc (rho={rho:.1f} m) {sweep_to_drop * rho:.1f} m "
                    f"of arc short of the drop point; coming in tangentially, clockwise."
                )
            else:
                return entry

        # Signed remaining sweep: positive while short of the drop point, negative once
        # the TRACTOR is past it. The modulo form above cannot express "past" -- it turns a
        # small overshoot into nearly 2*pi -- and both cases below need to tell the
        # difference.
        signed_sweep = wrap_to_pi(phi_drop - phi_here)

        # How far the reference that actually has to arrive trails the tractor. Arrival is
        # judged at the trailer's rear axle (see drop_reference), so the tractor has to
        # carry on roughly a trailer-length PAST the drop point before the load is over it.
        # Aiming at the drop point through that stretch would ask the rover to turn back on
        # itself; the target is allowed to run on around the arc by exactly that much
        # instead, so the last few metres stay path-following and the heading stays
        # tangential while the bed tips.
        ref_x, ref_y = self.drop_reference()
        trail = math.hypot(ref_x - self.state.x, ref_y - self.state.y)

        # Genuinely past it, by more than the trailing distance: the run-in is over and
        # there is nothing left to follow. Steer at the drop point and let the stop line in
        # in_drop_band accept the arrival.
        if signed_sweep < -(trail / rho) - 1e-9:
            return drop

        lookahead = abs(float(self.get_parameter("approach_lookahead_m").value))
        step = max(0.0, min(lookahead / rho, signed_sweep + trail / rho))
        return on_arc(phi_here + step)

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
            drop = self.drop_point()
            if drop is None or self.state is None:
                return 0.0
            return math.hypot(drop[0] - self.state.x, drop[1] - self.state.y)
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
        drop = self.drop_point()
        if self._approach_committed and drop is not None and self.state is not None:
            cx = float(self.get_parameter("site_center_x").value)
            cy = float(self.get_parameter("site_center_y").value)
            radius = math.hypot(drop[0] - cx, drop[1] - cy)
            if radius > 1e-6:
                home_angle = math.atan2(drop[1] - cy, drop[0] - cx)
                ref_x, ref_y = self.drop_reference()
                here_angle = math.atan2(ref_y - cy, ref_x - cx)
                # SIGN: the run-in now arrives CLOCKWISE, so the rover's site angle
                # DECREASES towards the drop point and it approaches from the
                # counter-clockwise side. Short of the drop point here_angle > home_angle,
                # so this is negative; it turns positive on crossing. The old
                # counter-clockwise run-in had exactly the opposite sign, and reusing that
                # test here would report arrival the instant the rover committed, tens of
                # metres out.
                passed_by = wrap_to_pi(home_angle - here_angle) * radius
                # Radial slack is looser here than the band, because having crossed the
                # line the rover is where it was going to be and stopping it is better
                # than sending it round again.
                if passed_by >= 0.0 and radial_err <= 2.0 * band:
                    return (
                        True,
                        f"passed the drop point by {passed_by:.2f} m of arc, "
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

        drop = self.drop_point() or self.home
        # The TRAILER AXLE against the drop point, not the tractor. This line used to
        # measure self.state, and because acceptance is the loosest of the three tests
        # below, that shortcut fired the moment the tractor reached the drop point --
        # bypassing the axle-based band entirely and leaving the load ~3 m short, out of
        # the builder's reach. That was the whole failure in the previous run.
        ref_x, ref_y = self.drop_reference()
        distance = math.hypot(drop[0] - ref_x, drop[1] - ref_y)
        tolerance = max(0.1, float(self.get_parameter("home_tolerance_m").value))
        in_band, band_why = self.in_drop_band()

        # Acceptance is GEOMETRIC ONLY. Reach and heading are measured and reported, but
        # they must never withhold acceptance. Holding for them was a livelock generator:
        # a rover that is refused at the drop point falls through to the driving branch
        # below, where home_approach_target has nothing left to follow and returns the
        # drop point itself -- a point that is beside the rover and inside its turning
        # radius, and that keeps sliding counter-clockwise as the builder advances a slot.
        # Pure pursuit cannot converge on such a point; it orbits it. With a ~7 m turning
        # radius about a point on the 37 m ring, that orbit cuts clean through the 33 m
        # builder orbit and the 30 m work circle -- the rover drives a lap around the very
        # builder it is delivering to. That is the failure in_drop_band's own docstring
        # warns about, and it is worse than a drop that is a metre off.
        #
        # Drop LATER, do not drop CONDITIONALLY: the way to put the load in reach is
        # drop_reference_point = "gate", which judges arrival at the lip the load pours
        # over instead of at the tractor, so the rig carries on those last few metres
        # before it tips. Geometry decides when; the numbers below only say how it went.
        reach = self.reach_to_builder()
        reach_why = "reach unknown"
        accept = self.parked_at_home or distance <= tolerance or in_band
        if accept and reach is not None:
            lo = abs(float(self.get_parameter("drop_reach_min_m").value))
            hi = abs(float(self.get_parameter("drop_reach_max_m").value))
            reach_why = f"reach {reach:.2f} m in [{lo:.2f}, {hi:.2f}]"
            tol_deg = abs(float(self.get_parameter("drop_heading_tolerance_deg").value))
            trailer_yaw = self.trailer_heading()
            if tol_deg > 0.0 and trailer_yaw is not None:
                cx_site = float(self.get_parameter("site_center_x").value)
                cy_site = float(self.get_parameter("site_center_y").value)
                drop_bearing = math.atan2(drop[1] - cy_site, drop[0] - cx_site)
                # Tangent for a clockwise arrival: (sin, -cos) of the drop bearing.
                tangent_cw = math.atan2(-math.cos(drop_bearing), math.sin(drop_bearing))
                err_deg = abs(math.degrees(wrap_to_pi(trailer_yaw - tangent_cw)))
                reach_why += f", heading {err_deg:.1f} deg off tangent"
            if not (lo <= reach <= hi):
                self.get_logger().warn(
                    f"Dropping at {reach_why} from builder {self.builder_id}'s arm base; "
                    f"outside the window, so some rocks may land out of its pickup radius.",
                    once=True,
                )

        if accept:
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
                    f"Parked at drop point (axle {distance:.2f} m from it, {band_why}, "
                    f"{reach_why}, entered band at {self._drop_entry_speed:.3f} m/s); "
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
        # Not the drop point itself: the return leg follows an arc that stays outside the
        # collector circle and touches it only where the load is dropped, so the rover
        # arrives running ALONG the ring and never crosses the builder's orbit. See
        # home_approach_target.
        steering = self.compute_steering(self.home_approach_target())
        target_speed = self.turn_speed(self.actual_steering(), target_speed)
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
        # Same bounds as the home approach. This is the call site that had none at all.
        speed_command = self.compute_speed_command(
            self.turn_speed(self.actual_steering(), self.pickup_approach_target_speed(switch_radius)))
        speed_command.steering = steering
        self.command = self.ramp_command(speed_command)
        self.publish_command(self.command)

    def speed_trace(self, base_speed: float, capped: float, steering_norm: float) -> None:
        """Say which limiter is actually binding, once a second.

        Added because guessing cost a run. Top speed fell from 2.46-2.61 m/s (run
        20260825_025343, before any of this) to 1.57-1.86 m/s against an unchanged 3.0 m/s
        target, and the traction bound looked like the culprit -- it was not: fixing it to
        read the ramped command instead of the raw demand moved the measured top speed by
        less than 0.1 m/s. So print every candidate side by side and let the log say which
        one is holding the rover down.
        """
        period = max(0.0, float(self.get_parameter("speed_trace_period_s").value))
        if period <= 0.0:
            return
        now = time.monotonic()
        if self._last_speed_trace is not None and now - self._last_speed_trace < period:
            return
        self._last_speed_trace = now
        traction = self.traction_speed_limit(steering_norm)
        turn_cap = float(self.get_parameter("reverse_turn_speed_mps").value)
        target = max(0.0, float(self.get_parameter("target_speed_mps").value))
        if self._hard_turning and abs(capped - turn_cap) < 1e-6:
            binding = "hard-turn"
        elif traction < base_speed - 1e-6 and abs(capped - traction) < 1e-6:
            binding = "traction"
        elif base_speed < target - 1e-6:
            binding = "approach-ramp"
        else:
            binding = "none (asking for target)"
        self.get_logger().info(
            f"speed: actual={self.state.speed:5.2f} ramped_target={self.ramped_target_speed:5.2f} "
            f"asked={base_speed:5.2f} -> capped={capped:5.2f} | hard_turning={int(self._hard_turning)} "
            f"turn_cap={turn_cap:4.2f} traction_cap={traction:5.2f} "
            f"(ramped steer {steering_norm:+.2f}) | binding={binding}"
        )

    def turn_speed(self, steering_norm: float, base_speed: float) -> float:
        """Speed to drive at, given the steer angle actually commanded.

        A turn on regolith is bounded on BOTH sides and the old code only bounded one.

        Too fast and cornering demand v^2 * curvature exceeds what the soil will give.
        Too slow and an Ackermann vehicle produces no yaw moment at all: the tires just
        plow, the rover digs in, and the lateral load goes into the steering axle. Logged
        perf lines caught it doing exactly that -- `steer=-0.50 thr=0.72 speed=0.52`, three
        quarters throttle at half lock making half a metre per second.

        The reason it starved is that the hard-turn speed was applied as
        min(target_speed, reverse_turn_speed_mps), while the comment above it said "the
        turn runs at its own fixed speed". min() is not that: once the approach ramp had
        wound target_speed below reverse_turn_speed_mps, the turn inherited the ramp's
        speed instead of its own. So a hard turn taken while decelerating -- which is every
        turn at the end of a leg -- ran at whatever was left.
        """
        asked = base_speed
        if self._hard_turning:
            base_speed = float(self.get_parameter("reverse_turn_speed_mps").value)
        capped = min(base_speed, self.traction_speed_limit(steering_norm))
        self.speed_trace(asked, capped, steering_norm)
        return capped

    def traction_speed_limit(self, steering_norm: float) -> float:
        """Upper bound the regolith can hold at the steer angle the WHEELS ARE AT, m/s.

        THE ARGUMENT IS THE RAMPED COMMAND, NOT THE DEMAND, and that distinction is the
        whole cost of getting this wrong. Fed the raw pure-pursuit demand, the bound
        punishes the rover for an angle the ramp is deliberately preventing it from
        reaching: a demand of 1.0 caps the speed at sqrt(1.3/0.2747) = 2.2 m/s while the
        wheels are still passing through 0.2, generating a fifth of the cornering load the
        bound was computed for. Measured over 115 s against the two runs before this
        existed, at the same delta and the same 3.0 m/s target:

          before   max 3.55-4.02 m/s, mean while moving 1.27-1.35 m/s
          demand   max 1.63-1.86 m/s, mean while moving 0.80 m/s

        A 55% cut in top speed for a load that was never applied -- the front uprights
        peaked at 19.4 deg, 0.56 of full lock, so the demand that set the bound was never
        anywhere near what the wheels did. The rover then corners slowly, takes longer to
        converge on the lane, and holds a steering demand for longer, which lowers the
        bound again.

        THE BUG THIS EXISTS TO CLOSE. compute_steering is called from two places -- the
        home approach and the outbound/pickup path -- and the hard-turn speed cap was
        applied at only the first. So a rover that hit the hard-turn regime on its way OUT,
        which is what happens on the turn that starts a new cycle, took a 0.36 rad arc at
        the full target_speed_mps. Launched at 5 m/s that is v^2 * curvature =
        25 * tan(0.36)/2.5 = 3.8 m/s^2 of cornering demand against ~1.3 available: it
        bulldozes, the lateral load goes into the steering axle, and the axle locks.

        Deriving it from the traction budget rather than a fixed number means the limit
        follows whatever steer angle is actually commanded, so ordinary pure-pursuit
        corrections at cruise are covered too, not just the hard-turn case.
        """
        max_angle = max(1e-6, float(self.get_parameter("max_steering_angle_rad").value))
        wheelbase = max(1e-6, float(self.get_parameter("wheelbase_m").value))
        a_lat = max(1e-6, float(self.get_parameter("max_lateral_accel_mps2").value))
        curvature = abs(math.tan(clamp(abs(steering_norm), 0.0, 1.0) * max_angle)) / wheelbase
        if curvature <= 1e-6:
            return float("inf")
        return math.sqrt(a_lat / curvature)

    def actual_steering(self) -> float:
        """The steering the wheels are at, as far as this node can know it.

        The last ramped command, i.e. one tick old at 20 Hz. That is the number the
        traction bound has to be computed from -- see traction_speed_limit.
        """
        return self.command.steering

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
