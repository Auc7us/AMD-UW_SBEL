import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
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
class BuilderState:
    x: float
    y: float
    yaw: float
    speed: float


def aim_point_on_circle(
    center_x: float,
    center_y: float,
    radius: float,
    here_angle: float,
    lookahead_arc_m: float,
    counter_clockwise: bool,
) -> Tuple[float, float]:
    """A point ON the lane, a SHORT arc ahead of where the builder currently is.

    Short is the whole point. The hull's measured natural turn radius is 10.5 m against a
    33 m lane, so turning authority was never the limit -- it can always turn tighter than
    the lane needs. What killed the earlier versions was letting the error grow until the
    loop demanded a correction big enough to rail the steering, and railing locks a track
    (see steering_limit), which removes the forward motion correction depends on.

    A long lookahead is what lets the error grow: the aim point is far enough away that
    the hull slips and drifts on the way to it, so each correction is applied to a stale
    situation and the loop closes around a bigger and bigger excursion. A short lookahead
    keeps the error small, so the steering stays small and never rails, and the residual
    oscillation about the lane is a few tens of centimetres -- which the arm's reach band
    absorbs without noticing.

    Two nice consequences of shortening it: the chord offset inside the circle falls to
    L^2/8R (15 mm at L=2.5) and the equilibrium heading error to L/2R (1.7 deg), so the
    structural bias that made a 6 m lookahead fight the hull's own turn is gone too.

    Anchoring to the builder's own ANGULAR position keeps this stateless -- the target is
    always a bounded arc ahead on the lane, never across the site, and always at exactly
    `radius`, so following it pulls the hull back onto the lane.
    """
    direction = 1.0 if counter_clockwise else -1.0
    aim_angle = here_angle + direction * (lookahead_arc_m / max(1e-3, radius))
    return (
        center_x + radius * math.cos(aim_angle),
        center_y + radius * math.sin(aim_angle),
    )


class BuilderOrbitController(Node):
    """Drive one tracked builder around a concentric circular path."""

    def __init__(self) -> None:
        super().__init__("builder_orbit_controller")

        self.declare_parameter("builder_id", 1)
        # CONVEYOR SPACING. How many builders share this orbit, and how much arc this one
        # must leave to the machine in front of it.
        #
        # The sector cap used to make this unnecessary: each builder owned 0.7 of its own
        # 2*pi/N and was forbidden to leave it, so two of them could not meet. Once a
        # builder is allowed to walk out of its sector and carry on into the one ahead --
        # laying the next layer over what its neighbour just finished -- that fence is
        # gone, and the only thing keeping them apart is not catching up.
        #
        # It has to live here, in the controller, and not in the sim. Cross-rank visibility
        # through SynChrono is not available in the configuration we run: draws_zombies is
        # `is_sensor_rank && !no_sensor`, every run passes --no_sensor, so no rank ever
        # builds a zombie body for another rank's builder and the remote agents are
        # SynQuietAgents that drop their messages. ROS 2 has the visibility for free -- every
        # builder already publishes /builder_N/vehicle_state on a shared DDS domain.
        #
        # 9 m at the 33 m orbit is a little under two hull lengths (5.40 m). At N=16 the
        # natural spacing is 12.96 m, so a builder may close up by 4 m before it is held.
        self.declare_parameter("num_builders", 0)          # 0 = infer from builder_id only
        self.declare_parameter("min_builder_gap_m", 9.0)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("work_circle_radius_m", 30.0)
        self.declare_parameter("path_radius_m", 33.0)
        self.declare_parameter("counter_clockwise", True)
        # waypoint_spacing_m is dead, still DECLARED so existing launch files keep working.
        self.declare_parameter("waypoint_spacing_m", 2.0)
        self.declare_parameter("lookahead_m", 2.5)
        self.declare_parameter("min_lookahead_m", 1.2)
        # Steering is proportional to heading error, in units of full-scale steering per
        # radian. 1.2 gives full lock at ~48 deg of error.
        self.declare_parameter("steering_kp", 1.2)
        # INTEGRAL action on heading error, and the builder cannot follow its lane without
        # it. Open-loop identification of this vehicle, four separate sims from an
        # identical spawn pose, 8 s of sim each, throttle 0.45:
        #
        #     steering   yaw_rate      travelled
        #       -0.60    -0.1155 rad/s   4.19 m
        #        0.00    +0.1191 rad/s  10.00 m     <-- turns hard LEFT when told straight
        #       +0.25    +0.2288 rad/s   4.58 m
        #       +0.60    +0.2133 rad/s   4.75 m     (saturating past ~0.25)
        #
        # so yaw_rate ~= 0.119 + 0.392 * steering. The map is monotonic with POSITIVE
        # slope -- the sign convention is correct and was never the problem -- but at zero
        # steering the hull yaws +0.119 rad/s, while holding a 33 m circle at the measured
        # 1.25 m/s needs only 0.038 rad/s. It turns 3.1x too hard inward on its own,
        # from asymmetric track drag (idler tensioner loads differ left-to-right by ~60%).
        # Trimming that needs a STANDING steering of about -0.21.
        #
        # A proportional-only law cannot supply it. Pure pursuit at lookahead L has an
        # equilibrium heading error of about +L/2R = +5.2 deg (aim point inboard of the
        # tangent), so the P term outputs +0.11 at equilibrium -- the wrong sign -- which
        # ADDS to the excess inward yaw. That is the monotonic inward spiral, with steering
        # pinned at the rail: radius 33 -> 28.7 m and heading error +0.6 -> +70 deg.
        #
        # The integrator holds whatever standing trim the vehicle actually needs at zero
        # heading error, which is exactly the missing degree of freedom. It also tracks the
        # bias as it changes with speed and terrain, which a hard-coded trim would not.
        self.declare_parameter("steering_ki", 0.9)
        # Anti-windup clamp on the integral state, in rad*s. With ki=0.9 this bounds the
        # integral's contribution to +/-0.9 of full-scale steering -- comfortably more than
        # the ~0.21 trim needed, without letting it run away while the vehicle is blocked.
        self.declare_parameter("steering_i_limit", 1.0)
        # Cross-track gain: how much heading to give up per metre of radial error, and the
        # cap on that correction. 0.35 rad/m means 1 m inside the lane asks for 20 deg of
        # outward heading; the 0.7 rad cap stops a badly displaced builder from trying to
        # drive straight at the lane and losing its tangential heading entirely.
        self.declare_parameter("radial_kp_rad_per_m", 0.35)
        self.declare_parameter("radial_corr_limit_rad", 0.40)
        # FEEDFORWARD, from the identified plant. This is the term that actually makes the
        # builder able to follow its lane, and no amount of feedback tuning substitutes
        # for it.
        #
        # Re-express the measured map in CURVATURE, which makes it speed-independent:
        #   kappa_bias  = 0.119 rad/s / 1.25 m/s = 0.0952 1/m   (10.5 m natural turn)
        #   kappa_gain  = 0.392 / 1.25           = 0.314 1/m per unit steering
        # A 33 m lane needs kappa = 1/33 = 0.0303 1/m, so
        #   steering_ff = (0.0303 - 0.0952) / 0.314 = -0.207
        # and because both the bias and the authority scale with speed, that number is a
        # constant rather than a function of how fast the builder happens to be going.
        #
        # Three feedback-only attempts failed here, and this is why: a PI starting from
        # zero has to DISCOVER -0.207 while the hull is already turning inward at
        # 0.119 rad/s -- 34 degrees lost in the first five seconds. Every run then spent
        # its time recovering from that transient instead of tracking. With feedforward
        # the loop starts at the right operating point and the PI only handles residuals.
        self.declare_parameter("hull_bias_curvature_per_m", 0.0952)
        self.declare_parameter("steering_curvature_gain_per_m", 0.314)
        # NEVER RAIL THE STEERING. This is the constraint that every earlier version of
        # this controller violated, and it is why they all failed the same way.
        #
        # On a skid-steer, steering and speed share the same actuators: Chrono's
        # ChTrackDrivelineBDS does `braking_left += steering`, so steering = 1.0 means one
        # track is FULLY LOCKED. The hull can then only pivot -- it has lost the forward
        # motion that correcting a path error requires. So the instant the controller asks
        # for a large correction it destroys its own ability to make one, and the run ends
        # in a trap: off the lane -> large error -> rail -> pivot in place -> further off
        # the lane. Measured at the end of three separate runs, steering pinned at +/-1.0
        # with the hull at 0.05-0.28 m/s and heading errors of 60-157 degrees.
        #
        # The identified map shows where the usable band ends:
        #   steering 0.00 -> 1.25 m/s      steering 0.25 -> 0.57 m/s
        #   steering 0.60 -> 0.52 m/s      steering 1.00 -> effectively pivot-only
        # so moderate steering keeps most of the speed. Capping well inside the rail keeps
        # the vehicle translating, which is the only state in which it can converge.
        # Convergence is slower; it actually happens.
        self.declare_parameter("steering_limit", 0.5)
        # Raised from 0.5. At 0.5 the steering command needs 2 s to go from straight to
        # full lock, which at the 2 m/s some runs use is 4 m of travel -- long enough to
        # leave a 33 m circle before the steering ever arrives. The ramp exists to stop
        # step changes into a tracked driveline, not to be the slowest term in the loop.
        self.declare_parameter("steering_ramp_per_s", 2.0)
        self.declare_parameter("target_speed_mps", 0.9)
        # The speed loop has to be AGGRESSIVE, because on a skid-steer it is fighting the
        # steering. Steering brakes one track, so every correction costs speed -- and
        # brake-steering authority scales WITH speed, so losing it is self-reinforcing:
        # the hull slows, loses the authority to turn back onto the lane, drifts further
        # off, asks for more steering, and slows further. Measured in that state:
        #   steering -0.5, throttle 0.29 -> 0.13 m/s   (identification: 1.25 m/s at
        #   steering -0.5, throttle 0.37 -> -0.07 m/s   throttle 0.45, steering 0)
        # With kp=0.6 and max 0.6, a hull at 0.13 m/s against a 0.6 target asked for only
        # 0.28 throttle -- nowhere near enough to hold speed against its own steering
        # brake. It must be allowed to use the whole pedal.
        self.declare_parameter("speed_kp", 2.0)
        self.declare_parameter("speed_tolerance_mps", 0.05)
        self.declare_parameter("max_throttle", 1.0)
        # How close to the station angle counts as arrived, and how much further it
        # must drift before setting off again. The gap between them is hysteresis: a
        # tracked vehicle cannot stop on a point, so without it the builder would
        # creep past, restart, and hunt around its station forever.
        #
        # SLOT PITCH, and the two station bands sized from it.
        #
        # These used to be absolute: an ANGLE for arrival, METRES for the keep deadband,
        # both hand-set for a 33 m lane at 0.9 m slot pitch. Scaling the site to a 53 m
        # lane at 0.5 m pitch left station_tolerance_rad = 0.015 untouched, and 0.015 rad
        # at r = 53 is 0.795 m of arc -- 1.59x the ENTIRE slot pitch. Every new station was
        # therefore already inside the arrival band the moment the sim published it: the
        # builder latched holding_station immediately, never entered the drive branch, and
        # the forward creep to the next slot -- the only thing that corrects radial error
        # (see the radial note further down) -- never ran once. Measured on
        # run_20260827_061917: all eight builders spiralled 6-7 m inside the lane, took
        # station off-lane, and starved. Twenty rocks laid in 600 s.
        #
        # So they are FRACTIONS of the slot pitch now, and the pitch itself is measured off
        # the station stream rather than duplicated from RobotLayout.h. Change the site
        # radius or the pitch and these follow, with no second place to forget.
        #
        #   arrival  0.55 * pitch -- the ratio the working 33 m / 0.9 m config had
        #                            (0.495 m of 0.900 m). It MUST stay below 1.0: at 1.0
        #                            arrival is free and the creep never happens.
        #   deadband 0.28 * pitch -- likewise (0.250 m of 0.900 m).
        #
        # wall_slot_pitch_m is a BOOTSTRAP only, for the first station before any advance
        # has been seen. Keep it equal to RobotLayout.h's wall_slot_pitch_m; if it drifts,
        # the measured value silently wins and says so in the log.
        self.declare_parameter("wall_slot_pitch_m", 0.5)
        self.declare_parameter("station_tolerance_frac_of_pitch", 0.55)
        self.declare_parameter("station_keep_deadband_frac_of_pitch", 0.28)
        # Absolute OVERRIDES for the two bands. 0.0 means "derive from the pitch", which is
        # what every launch file should use. A positive value pins the band and is reported
        # at startup, because pinning it is how the failure above happened.
        self.declare_parameter("station_tolerance_rad", 0.0)
        # Widened from 0.20 rad. With active station keeping (see on_timer) the
        # builder now corrects creep continuously instead of waiting to fall out of
        # this band, so the band's only remaining job is to catch a builder that has
        # been shoved so far it is genuinely quicker to drive round than to creep
        # back. 0.6 rad is ~20 m of arc at the 33 m path radius.
        self.declare_parameter("station_release_rad", 0.60)
        # Arrival is NOT an angle-only test. station_error is an orbit angle, so it is
        # satisfied anywhere along the radius -- a builder 3 m inside the lane, at the
        # right bearing, reads as "on station". It then stops driving, and because it is
        # off the lane the aim point still demands a large heading change, so it steers
        # with almost no throttle: a skid-steer pivot, grinding itself round in place and
        # further off the lane. That is the inward turn seen at the first pickup point,
        # and the angle-only test is what let it start.
        # Widened from 0.6. Oscillating about the lane by a few tens of centimetres is
        # fine and expected with a short lookahead; what is NOT fine is refusing to take
        # station because of it, which leaves the arm without a pick forever. The hard
        # limit is physical: the wall slot sits 3.095 m from the arm base on the lane, and
        # LrvArm rejects a grab closer than 2.0 m, so the builder can be about 1.09 m
        # inside the lane before its own wall slot goes out of reach. 0.9 keeps margin.
        # How far off the lane the builder may be and still TAKE station.
        #
        # Sized by the arm, not by taste: on station the wall slot sits 3.095 m from the
        # arm base and LrvArm refuses a grab closer than 2.0 m, so at 1.09 m inside the
        # lane the builder can no longer reach the slot it is there to fill. 0.9 keeps a
        # little margin under that.
        self.declare_parameter("station_radius_tol_m", 0.9)
        # Purely a reporting threshold now -- see the radial note in on_timer. Nothing in
        # the control law reacts to it, so it is set where a drift becomes worth knowing
        # about rather than where it becomes worth acting on.
        self.declare_parameter("radius_warn_m", 1.5)
        # Radial band at which an ALREADY-HELD station is given up, as opposed to the band
        # required to take one. Hysteresis, and it is what stops one builder laying a
        # quarter of what its neighbours lay.
        #
        # Measured on rank 3, terrain tilt 5.2 deg: "holding station at 186.1 deg", then
        # 0.7 s of sim later "-0.90 m off the 33.0 m lane". A single band means the drift
        # that follows parking immediately un-parks the builder and sends it round to
        # re-acquire -- and un-parking releases the sim-side anchor that would have pulled
        # it back, so the correction it needs is the thing that dropping station keeping
        # switches off. It was not slow at anything; it kept being sent away.
        #
        # 0.4 m of hysteresis, and both ends of it are constrained.
        #
        # It cannot be much tighter. At 1.05 the release band sat 0.15 m from the take
        # band, which is inside the drift the builder picks up in a single 1 m creep, so
        # rank 3 chattered across it -- take station, drift, give it up, drive a lap,
        # repeat. That is what had one builder laying a quarter of what its neighbours did.
        #
        # It cannot be much looser either, and 3.0 proved that the expensive way: builder 3
        # drifted to -3.00 m before anything reacted, which is r = 30.0 -- it had driven
        # onto the work circle, on top of the wall it had just laid. The hull is 2.686 m
        # wide, so the true ceiling is about 1.5 m.
        #
        # The builder cannot correct radial error while parked -- see the anchor note in
        # BuilderRig::Synchronize, which cannot beat 15.3 kN of braked-track friction -- so
        # giving up station and driving the lane IS the correction. This band decides when
        # that is worth a lap, and 1.3 m is where the arm has just stopped being able to
        # reach its own slot.
        self.declare_parameter("station_radius_release_m", 1.3)
        # Active station keeping. Inside the deadband the builder sits on the brake;
        # outside it, it creeps forward along its arc at gain * error, capped.
        self.declare_parameter("station_keep_deadband_m", 0.0)   # 0 = derive from pitch
        self.declare_parameter("station_keep_speed_mps", 0.5)
        self.declare_parameter("station_keep_gain", 0.35)
        # FLOOR on the creep speed, and it is not optional. gain * (error - deadband)
        # goes to zero as the builder approaches the deadband, so just outside it the
        # commanded speed is a few mm/s -- which for an M113 is no throttle at all. It
        # stalls a hair outside the band, never publishes the full brake that means "on
        # station", and the arm is never offered a pick. Observed live: a builder parked
        # 0.007 m outside a 0.12 m deadband, indefinitely. Better to creep at a speed the
        # vehicle can actually make, overshoot slightly, and brake: the wall slots are
        # fixed world points, so a little overshoot costs reach, not accuracy.
        self.declare_parameter("station_keep_min_speed_mps", 0.25)
        # Steering authority cap during the fine approach. A skid-steer turns by driving
        # its tracks in opposition, so steering with little or no throttle is a PIVOT --
        # it rotates the hull and skids it sideways without going anywhere. That is what
        # put four builders 2.5-3 m inside a 33 m lane at ~85 deg off tangential: they
        # stalled just outside the deadband on almost no throttle while still commanding
        # steering, and simply ground themselves round and inward. The speed floor above
        # is the primary fix; this bounds the damage if the builder is ever slow here
        # again.
        # 0.0 = derive, which is what it should be. The cap has to clear the STANDING
        # TRIM this lane needs, |(1/R - hull_bias)/gain|, or it throttles the correction
        # exactly when the builder is off-lane and needs it. At R = 53 that trim is 0.243
        # of full steering, so the old fixed 0.35 left only 0.107 for correction while the
        # pursuit law was asking for 0.48 -- a third of the authority discarded, and the
        # hull's uncancelled turn curled it inward. Derived: |ff| + margin, capped by
        # steering_limit, so it scales with the lane instead of being re-guessed.
        self.declare_parameter("station_keep_max_steering", 0.0)
        self.declare_parameter("station_keep_steer_margin", 0.15)
        #
        # SPEED GATE ON STEERING. The one that matters -- read this before touching the
        # steering gains.
        #
        # This control law treats steering as a CURVATURE command: hull_bias_curvature_per_m,
        # steering_curvature_gain_per_m, kappa_wanted = direction/radius. A skid-steer's
        # steering is not curvature, it is a track-speed DIFFERENTIAL, which is a YAW RATE
        # command. The two agree only while the hull is translating:
        #
        #     curvature   kappa = dtheta/ds   -> ds = 0 gives dtheta = 0
        #     skid-steer  dtheta/dt != 0 with ds = 0  -> pivot in place, kappa unbounded
        #
        # So the 0.243 of standing trim that correctly holds a 53 m lane at 0.9 m/s is, on a
        # hull making 0.03 m/s, a pure pivot command -- and a pivoting hull's centre swings
        # on a radius of one hull half-length, which walks it off the lane.
        #
        # Measured on run_20260827_155242 rank 3, t=115..200: yaw swept 46 deg, lateral
        # velocity 3x forward velocity, v/omega = 2.64 m against a 2.70 m half-length, and
        # the entire 1.84 m radial excursion is 2.7*(sin 47.5 - sin 1.6) = 1.91 m of swing.
        # It rotated in place and travelled nowhere. Raising the station-keep cap made this
        # WORSE, because the cap was never the variable.
        #
        # The gate scales the steering DEMAND by how fast the hull is actually going, so at
        # rest it asks for nothing and all the effort goes into translating. Once moving,
        # full authority returns and the curvature model is valid again.
        #
        # The knee separates the two regimes cleanly as measured: the pivot ran at 0.003 to
        # 0.05 m/s, while a legitimate creep is floored at station_keep_min_speed_mps = 0.25.
        # 0.15 m/s sits between them, so this costs a real creep nothing.
        #
        # What it gives up is worth naming: below the knee the hull drives straight instead of
        # following the lane's curvature. Over one slot pitch that is L^2/(8R) = 0.53^2/424 =
        # 0.66 MM of chord error. Steering at creep speed buys 0.66 mm and costs 1.9 m.
        self.declare_parameter("steering_speed_knee_mps", 0.15)
        #
        # GIVE UP AND HOLD. A creep that is not converging must end in the brake, not in
        # more steering.
        #
        # Brake-and-hold was always implemented -- it is the `arc_error <= deadband` branch
        # below -- but nothing ever decided to USE it when the approach failed. Rank 3 of
        # run_20260827_155242 sat 0.53 m short of its station and stayed in the creep branch
        # for 85 s, which is how it came to rotate 46 deg in place.
        #
        # Holding short is nearly free, which is the point: from 0.53 m behind the station
        # the arm's reach to its own slot is hypot(3.095, 0.53) = 3.14 m, against a 2.0-5.2 m
        # envelope. The builder lays from where it stands, and the arm's own reach test --
        # not this controller -- decides whether that is good enough. If it is not, the
        # sim's unservable-slot timeout advances the station and the builder sets off again
        # with a fresh target, which is a FORWARD move and costs no lap.
        #
        # Progress is judged on arc to the station, not on speed: a hull pivoting on the
        # spot has non-zero speed and makes no progress, which is exactly the case that has
        # to be caught.
        self.declare_parameter("station_creep_stall_s", 5.0)
        self.declare_parameter("station_creep_progress_m", 0.02)

        self.builder_id = int(self.get_parameter("builder_id").value)
        self.state_topic = f"/builder_{self.builder_id}/vehicle_state"
        self.command_topic = f"/builder_{self.builder_id}/vehicle_cmd"
        self.station_topic = f"/builder_{self.builder_id}/station_angle"
        self.arm_status_topic = f"/builder_{self.builder_id}/arm_status"
        # The builder AHEAD. Travel is counter-clockwise and rank i sits at ray 2*pi*i/N,
        # so the next higher id is the one in front; the highest wraps to the lowest.
        n = int(self.get_parameter("num_builders").value)
        self.ahead_id = (self.builder_id % n) + 1 if n > 1 else None
        self.ahead_state: Optional[BuilderState] = None
        self.holding_for_ahead = False
        # Angular slot pitch as MEASURED from consecutive station advances. Running
        # minimum, not the latest: the sim may skip a slot (the unservable-slot timeout
        # advances the station by more than one pitch), and a multiple of the pitch would
        # widen both bands. The minimum positive step it ever takes is the pitch.
        self.slot_pitch_rad_measured: Optional[float] = None
        self.reported_measured_pitch = False
        self.reported_speed_gate = False
        # Creep progress watch: best arc-to-station seen on this approach, and how long since
        # it last improved. Reset whenever the station moves.
        self.creep_best_arc = None
        self.creep_stalled_s = 0.0
        self.creep_gave_up = False
        self.state: Optional[BuilderState] = None
        self.steering_command = 0.0
        # Integral of heading error, in rad*s. Holds the standing steering trim the hull
        # needs to travel straight; see steering_ki.
        self.steering_integral = 0.0
        # Angle on the orbit this builder should wait at, published by the sim. It steps
        # ONE WALL SLOT -- about 0.99 m of lane -- each time this builder lays a rock, and
        # is not tied to its collector's harvest cycle in any way. Older note said it
        # stepped per dump; that was the previous behaviour and is no longer true.
        self.station_angle: Optional[float] = None
        self.holding_station = False
        self.reported_off_lane = False
        # The arm is mid pick-and-place. While it is, this controller does not move the
        # builder AT ALL -- not even station-keeping creep.
        #
        # The grab is solved against the arm base frame, and that frame is bolted to this
        # hull. A builder that creeps during the ~15 s of a pick sends the gripper to
        # where the rock was when the pose was solved. Seen live: a builder whose station
        # keeping was still hunting started a pick, then accelerated to 0.67 m/s, and the
        # arm reported error_code 3 -- fingers closed on empty air.
        #
        # There is nothing to chase anyway: the sim only advances the station angle once a
        # rock has been laid, so during a pick the target is not moving.
        self.arm_busy = False

        self.command_pub = self.create_publisher(
            Float64MultiArray, self.command_topic, 10
        )
        self.create_subscription(
            Float64MultiArray, self.state_topic, self.on_state, 10
        )
        self.create_subscription(
            Float64MultiArray, self.station_topic, self.on_station_angle, 10
        )
        self.create_subscription(
            Float64MultiArray, self.arm_status_topic, self.on_arm_status, 10
        )
        if self.ahead_id is not None:
            self.create_subscription(
                Float64MultiArray, f"/builder_{self.ahead_id}/vehicle_state",
                self.on_ahead_state, 10
            )
            self.get_logger().info(
                f"conveyor: following builder {self.ahead_id}, holding "
                f"{float(self.get_parameter('min_builder_gap_m').value):.1f} m of arc."
            )

        rate_hz = max(
            1.0, float(self.get_parameter("control_rate_hz").value)
        )
        self.dt = 1.0 / rate_hz
        self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"builder_{self.builder_id} orbit controller: "
            f"path radius={self.get_parameter('path_radius_m').value:.1f} m, "
            f"tangent-following, radial gain "
            f"{self.get_parameter('radial_kp_rad_per_m').value:.2f} rad/m, "
            f"steering PI kp={self.get_parameter('steering_kp').value:.2f} "
            f"ki={self.get_parameter('steering_ki').value:.2f}, "
            f"{self.state_topic} -> {self.command_topic}"
        )
        # Report the DERIVED bands in metres, which is the form the failure was visible in.
        # An arrival band at or above one slot pitch means the builder never has to drive to
        # a station, and the creep that corrects radial error never happens -- so say the
        # ratio out loud at startup rather than leaving it to be worked out after a run.
        radius = max(1e-3, float(self.get_parameter("path_radius_m").value))
        pitch_m = self.slot_pitch_rad() * radius
        tol_m = self.station_tolerance_rad() * radius
        pinned = []
        if float(self.get_parameter("station_tolerance_rad").value) > 0.0:
            pinned.append("station_tolerance_rad")
        if float(self.get_parameter("station_keep_deadband_m").value) > 0.0:
            pinned.append("station_keep_deadband_m")
        if float(self.get_parameter("station_keep_max_steering").value) > 0.0:
            pinned.append("station_keep_max_steering")
        self.get_logger().info(
            f"stations: pitch {pitch_m:.3f} m (bootstrap), arrival {tol_m:.3f} m "
            f"({tol_m / max(1e-6, pitch_m):.2f} of pitch), keep deadband "
            f"{self.station_keep_deadband_m():.3f} m"
            + (f" -- PINNED by {', '.join(pinned)}" if pinned else "")
        )
        if tol_m >= pitch_m:
            self.get_logger().error(
                f"arrival band {tol_m:.3f} m is not smaller than the {pitch_m:.3f} m slot "
                f"pitch: every station will read as already reached, the builder will never "
                f"drive to one, and radial error will never be corrected."
            )

    def on_state(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().warn(
                "Ignoring vehicle_state; expected [x, y, yaw, speed]."
            )
            return
        self.state = BuilderState(
            x=float(msg.data[0]),
            y=float(msg.data[1]),
            yaw=float(msg.data[2]),
            speed=float(msg.data[3]),
        )

    def on_station_angle(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        angle = float(msg.data[0])
        if self.station_angle is None or abs(wrap_to_pi(angle - self.station_angle)) > 1e-6:
            if self.station_angle is not None:
                self.note_station_step(wrap_to_pi(angle - self.station_angle))
            self.station_angle = angle
            self.creep_best_arc = None
            self.creep_stalled_s = 0.0
            self.creep_gave_up = False
            if self.holding_station:
                self.holding_station = False
                self.get_logger().info(
                    f"station moved to {math.degrees(angle):.1f} deg; driving to the new one."
                )

    def on_ahead_state(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 4:
            self.ahead_state = BuilderState(float(msg.data[0]), float(msg.data[1]),
                                            float(msg.data[2]), float(msg.data[3]))

    def note_station_step(self, step: float) -> None:
        """Learn the slot pitch from how far the station just moved.

        Sanity-bounded: a step must be forward and must not exceed the bootstrap pitch by
        more than 4x, so a genuine multi-slot skip or a re-seeded station cannot poison
        the bands. Stations only ever move forward (see on_station_angle in the sim), so a
        negative step means something is wrong and is ignored rather than trusted.
        """
        if not bool(self.get_parameter("counter_clockwise").value):
            step = -step
        if step <= 1e-6:
            return
        bootstrap = self.bootstrap_pitch_rad()
        if step > 4.0 * bootstrap:
            return
        if self.slot_pitch_rad_measured is None or step < self.slot_pitch_rad_measured:
            self.slot_pitch_rad_measured = step
            radius = max(1e-3, float(self.get_parameter("path_radius_m").value))
            if not self.reported_measured_pitch:
                self.reported_measured_pitch = True
                self.get_logger().info(
                    f"measured slot pitch {step * radius:.3f} m of arc "
                    f"({math.degrees(step):.4f} deg); arrival band "
                    f"{self.station_tolerance_rad() * radius:.3f} m, keep deadband "
                    f"{self.station_keep_deadband_m():.3f} m."
                )

    def bootstrap_pitch_rad(self) -> float:
        radius = max(1e-3, float(self.get_parameter("path_radius_m").value))
        return max(1e-6, abs(float(self.get_parameter("wall_slot_pitch_m").value))) / radius

    def slot_pitch_rad(self) -> float:
        """Angular slot pitch: measured when we have it, bootstrap parameter until then."""
        if self.slot_pitch_rad_measured is not None:
            return self.slot_pitch_rad_measured
        return self.bootstrap_pitch_rad()

    def station_tolerance_rad(self) -> float:
        """Arrival band, as an angle. A FRACTION OF THE SLOT PITCH -- see the note on
        station_tolerance_frac_of_pitch. Never allowed to reach a whole pitch, because at
        one pitch every station is pre-arrived and the builder stops creeping entirely."""
        override = float(self.get_parameter("station_tolerance_rad").value)
        if override > 0.0:
            return abs(override)
        frac = abs(float(self.get_parameter("station_tolerance_frac_of_pitch").value))
        return min(0.9, frac) * self.slot_pitch_rad()

    def station_keep_deadband_m(self) -> float:
        """Keep deadband, in metres of arc. Also a fraction of the slot pitch."""
        override = float(self.get_parameter("station_keep_deadband_m").value)
        if override > 0.0:
            return abs(override)
        frac = abs(float(self.get_parameter("station_keep_deadband_frac_of_pitch").value))
        radius = max(1e-3, float(self.get_parameter("path_radius_m").value))
        return min(0.9, frac) * self.slot_pitch_rad() * radius

    def station_keep_steer_cap(self, steering_ff: float) -> float:
        """Steering authority during the fine approach.

        Derived so it always clears this lane's standing trim: a cap below |steering_ff|
        means the builder cannot even hold the lane's curvature while creeping, let alone
        correct a radial error. Bounded above by steering_limit, which is the real physical
        constraint (full steering locks a track and the hull stops translating)."""
        limit = abs(float(self.get_parameter("steering_limit").value)) or 1.0
        override = float(self.get_parameter("station_keep_max_steering").value)
        if override > 0.0:
            return min(abs(override), limit)
        margin = abs(float(self.get_parameter("station_keep_steer_margin").value))
        return min(limit, abs(steering_ff) + margin)

    def gap_to_ahead(self) -> Optional[float]:
        """Arc from this builder to the one in front, along the direction of travel.

        None when there is nobody in front or nothing has been heard from them yet -- and
        None means "no constraint", so a silent neighbour never stalls this builder. That
        is the right failure direction here: the topic going quiet is far more likely than
        two machines actually converging, and a run that halts because a neighbour crashed
        is worse than one that keeps building.
        """
        if self.ahead_state is None or self.state is None:
            return None
        cx = float(self.get_parameter("center_x").value)
        cy = float(self.get_parameter("center_y").value)
        radius = max(1e-3, float(self.get_parameter("path_radius_m").value))
        mine = math.atan2(self.state.y - cy, self.state.x - cx)
        theirs = math.atan2(self.ahead_state.y - cy, self.ahead_state.x - cx)
        delta = theirs - mine
        if not bool(self.get_parameter("counter_clockwise").value):
            delta = -delta
        # Forward-only: they are always AHEAD, so wrap into [0, 2*pi) rather than [-pi, pi).
        delta %= 2 * math.pi
        return delta * radius

    def on_arm_status(self, msg: Float64MultiArray) -> None:
        """Track whether the arm is mid pick-and-place. State 1 == BUSY (LrvArm)."""
        if len(msg.data) < 2:
            return
        busy = int(round(float(msg.data[1]))) == 1
        if busy != self.arm_busy:
            self.arm_busy = busy
            self.get_logger().info(
                "arm started a pick; holding the hull still."
                if busy
                else "arm finished; free to move to the next slot."
            )

    def station_error(self) -> Optional[float]:
        """Signed angle still to travel to the station, along the direction of travel.

        Measured the way the builder actually moves: it can only go around its orbit,
        so an error of -10 deg against counter-clockwise travel means nearly a full
        lap, not a short reverse.
        """
        if self.state is None or self.station_angle is None:
            return None
        here = math.atan2(
            self.state.y - float(self.get_parameter("center_y").value),
            self.state.x - float(self.get_parameter("center_x").value),
        )
        error = wrap_to_pi(self.station_angle - here)
        if not bool(self.get_parameter("counter_clockwise").value):
            error = -error
        return error

    def on_timer(self) -> None:
        if self.state is None:
            self.publish_command(0.0, 0.0, 1.0)
            return

        # CONVEYOR HOLD. Stand still rather than close on the machine in front.
        #
        # Checked before anything else that can command motion, and it stops the hull only
        # -- the arm is untouched, so a held builder carries on laying whatever is in reach
        # instead of idling. That is the whole point of the pattern: the wall gets built
        # from wherever you are standing, and you move up when there is room.
        gap = self.gap_to_ahead()
        min_gap = abs(float(self.get_parameter("min_builder_gap_m").value))
        if gap is not None and gap < min_gap:
            if not self.holding_for_ahead:
                self.holding_for_ahead = True
                self.get_logger().warn(
                    f"builder {self.ahead_id} is {gap:.2f} m ahead, inside the {min_gap:.1f} m "
                    f"gap; holding here until it moves on."
                )
            self.steering_command = approach(
                self.steering_command, 0.0,
                max(0.0, float(self.get_parameter("steering_ramp_per_s").value)) * self.dt)
            self.publish_command(self.steering_command, 0.0, 1.0)
            return
        if self.holding_for_ahead and gap is not None and gap >= min_gap:
            self.holding_for_ahead = False
            self.get_logger().info(f"gap to builder {self.ahead_id} reopened to {gap:.2f} m; moving again.")

        # Absolute freeze while the arm works. See self.arm_busy.
        if self.arm_busy:
            self.steering_command = approach(
                self.steering_command,
                0.0,
                max(0.0, float(self.get_parameter("steering_ramp_per_s").value)) * self.dt,
            )
            self.publish_command(self.steering_command, 0.0, 1.0)
            return

        radius = max(
            1e-3, float(self.get_parameter("path_radius_m").value)
        )

        error = self.station_error()
        here = math.atan2(
            self.state.y - float(self.get_parameter("center_y").value),
            self.state.x - float(self.get_parameter("center_x").value),
        )
        # Signed radial error against the lane: + is outside, - is inside.
        radius_error = math.hypot(
            self.state.x - float(self.get_parameter("center_x").value),
            self.state.y - float(self.get_parameter("center_y").value),
        ) - radius

        # Short-lookahead pure pursuit, plus the measured feedforward, plus a hard cap
        # that keeps the command away from the rail. Each piece is load-bearing:
        #   lookahead  -- short, so the error never grows big enough to need a big
        #                 correction (see aim_point_on_circle)
        #   feedforward-- supplies the standing trim the hull needs at zero error
        #                 (see hull_bias_curvature_per_m)
        #   cap        -- full steering locks a track and the hull stops translating
        #                 (see steering_limit)
        direction = 1.0 if bool(self.get_parameter("counter_clockwise").value) else -1.0
        lookahead = max(0.3, float(self.get_parameter("lookahead_m").value))
        if error is not None:
            remaining = wrap_to_pi(error) * radius
            if remaining > 0.0:
                lookahead = min(lookahead, max(float(self.get_parameter("min_lookahead_m").value),
                                               remaining))
        aim = aim_point_on_circle(
            float(self.get_parameter("center_x").value),
            float(self.get_parameter("center_y").value),
            radius,
            here,
            lookahead,
            bool(self.get_parameter("counter_clockwise").value),
        )
        bearing = math.atan2(aim[1] - self.state.y, aim[0] - self.state.x)
        head_err = wrap_to_pi(bearing - self.state.yaw)
        kp = float(self.get_parameter("steering_kp").value)
        ki = float(self.get_parameter("steering_ki").value)
        i_limit = abs(float(self.get_parameter("steering_i_limit").value))
        # Feedforward the steering this hull needs just to hold the lane's curvature. See
        # hull_bias_curvature_per_m.
        kappa_gain = float(self.get_parameter("steering_curvature_gain_per_m").value)
        steering_ff = 0.0
        if abs(kappa_gain) > 1e-6:
            kappa_wanted = direction / radius
            kappa_bias = float(self.get_parameter("hull_bias_curvature_per_m").value)
            steering_ff = clamp((kappa_wanted - kappa_bias) / kappa_gain, -1.0, 1.0)
        limit = abs(float(self.get_parameter("steering_limit").value)) or 1.0
        # See steering_speed_knee_mps. Steering only means curvature while the hull moves.
        knee = abs(float(self.get_parameter("steering_speed_knee_mps").value))
        speed_gate = 1.0 if knee <= 1e-6 else clamp(abs(self.state.speed) / knee, 0.0, 1.0)
        raw = (steering_ff + kp * head_err + ki * self.steering_integral) * speed_gate
        # Anti-windup against the ACTUAL limit, not against 1.0 -- the output saturates at
        # `limit`, so that is where accumulating has to stop. The reversal exception lets a
        # saturated builder still wind the integral back down; without it the integral that
        # saturated it stays frozen at the value that keeps it there.
        # Anti-windup against the ACTUAL limit, and against the GATE. While the gate is
        # closed the hull cannot act on head_err at all, so integrating it is winding up a
        # term that will snap in the moment the hull starts moving -- which is how a builder
        # would leave its station in a turn. Decay instead, the same way the on-station
        # branch does.
        if speed_gate < 0.999:
            self.steering_integral *= 0.98
        elif abs(raw) < limit or (raw > 0.0) != (head_err > 0.0):
            self.steering_integral = clamp(
                self.steering_integral + head_err * self.dt, -i_limit, i_limit
            )
        target_steering = clamp(raw, -limit, limit)
        steering_delta = max(
            0.0,
            float(self.get_parameter("steering_ramp_per_s").value),
        ) * self.dt
        self.steering_command = approach(
            self.steering_command, target_steering, steering_delta
        )

        # Hold at the station once reached: brake and stop steering. Reaching it is
        # judged on the ORBIT angle, not straight-line distance, because that is the
        # only direction the builder can travel.
        # Radial error is computed above, with the steering law that uses it. Part of the
        # arrival test too -- see station_radius_tol_m -- because being at the right
        # bearing is not the same as being in the right place, and the arm needs the place.
        # RADIAL ERROR NEVER COSTS A LAP. It is reported and nothing else.
        #
        # This used to be an arrival AND release test, and the release is what made a drift
        # expensive: give up station, drive all the way round, come back. On run
        # 20260825_221818 rank 2's builder drifted 1.56 m inward over 170 s of parked time,
        # released at t=285, and the lap back cost 380 s -- the run ended before it arrived.
        # With sixteen ranks on this orbit a lap is not merely slow, it drives one builder
        # through the next one's station. The whole arrangement only works if every builder
        # advances forward, one slot pitch at a time, and never doubles back.
        #
        # Correcting it in place does not work either, and was tried: pivoting a braked
        # tracked hull on regolith walks it further off the lane than the creep pulls it
        # back, measured going -0.40 -> -1.30 -> -2.10 -> -2.86 -> -3.38 -> -3.75 -> -3.95 m
        # over six attempts before the run ended. Reverted.
        #
        # What actually corrects it is the ordinary forward creep to the next slot: pure
        # pursuit aims at a point ON the lane, so every advance pulls the radius in. That
        # only stops happening when the builder stops advancing, which is starvation -- a
        # different bug, fixed on the sim side. So: take station on the ANGLE, let the arm's
        # own reach test decide whether it can work from here, and if it cannot, the
        # unservable-slot timeout advances the station and the builder creeps FORWARD out of
        # the problem.
        radius_tol = abs(float(self.get_parameter("station_radius_tol_m").value))
        off_lane_warn = abs(float(self.get_parameter("radius_warn_m").value))
        if abs(radius_error) > off_lane_warn and not self.reported_off_lane:
            self.reported_off_lane = True
            self.get_logger().warn(
                f"{radius_error:+.2f} m off the {radius:.1f} m lane, past the {off_lane_warn:.1f} m "
                f"reporting band. Working from here anyway; the creep to the next slot is what "
                f"pulls it back."
            )
        elif abs(radius_error) <= radius_tol and self.reported_off_lane:
            self.reported_off_lane = False
            self.get_logger().info(f"back on the lane ({radius_error:+.2f} m).")

        if error is not None:
            tolerance = self.station_tolerance_rad()
            # Arrival is judged on ABSOLUTE angular proximity, not on remaining
            # travel. Remaining travel is forced into [0, 2*pi), so overshooting
            # the station by one control tick makes it read ~2*pi -- "nearly a
            # full lap to go" -- and the builder commits to another whole orbit
            # instead of stopping the few centimetres past where it wanted to be.
            # Approaching from either side counts as being on station.
            if not self.holding_station and abs(wrap_to_pi(error)) <= tolerance:
                self.holding_station = True
                self.get_logger().info(
                    f"holding station at {math.degrees(self.station_angle):.1f} deg "
                    f"(radius {radius_error:+.2f} m)."
                )
            # NO RELEASE BRANCH. Station is given up in exactly one place -- on_station_angle,
            # when the sim advances the slot -- and the new station is always AHEAD. Anything
            # else releasing it hands the builder a lap, and station keeping already handles
            # both ways of being off the mark without one: it creeps forward if it is short,
            # and stands on the brake if it has crept past. station_release_rad is now
            # unused; it is left declared so an existing launch file does not fail on it.

        if self.holding_station:
            # Station keeping is ACTIVE, not just the brake.
            #
            # Holding used to mean brake=1.0 and nothing else, with a release band of
            # 0.20 rad -- 7 m of arc at this radius. On sloped regolith the terrain
            # simply pushed the builder back, and because nothing corrected inside the
            # band it drifted the whole 7 m before the controller reacted, then drove a
            # long arc to come back. From outside that reads as a builder that sits
            # still and occasionally rolls backwards, which is exactly what it was.
            #
            # So close the loop on position instead: measure how far off station it is
            # along its own arc and creep back onto the mark under pure-pursuit
            # steering, at a speed proportional to the error. Inside a small deadband
            # it holds the brake, so there is no hunting.
            arc_error = wrap_to_pi(error) * radius if error is not None else 0.0
            deadband = self.station_keep_deadband_m()

            # Watch the approach. See station_creep_stall_s.
            progress = abs(float(self.get_parameter("station_creep_progress_m").value))
            stall_limit = abs(float(self.get_parameter("station_creep_stall_s").value))
            if arc_error > deadband and not self.creep_gave_up:
                if self.creep_best_arc is None or arc_error < self.creep_best_arc - progress:
                    self.creep_best_arc = arc_error
                    self.creep_stalled_s = 0.0
                else:
                    self.creep_stalled_s += self.dt
                    if stall_limit > 0.0 and self.creep_stalled_s >= stall_limit:
                        self.creep_gave_up = True
                        self.get_logger().warn(
                            f"creep stalled {arc_error:.2f} m short of station after "
                            f"{stall_limit:.0f} s without progress; braking and holding here. "
                            f"The arm works from this spot or the slot times out."
                        )

            if arc_error <= deadband or self.creep_gave_up:
                # On the mark, crept slightly PAST it, or gave up short of it. Do not chase
                # forward past the station -- the builder only drives one way round, so
                # overshoot would cost a full lap. Sit on the brake.
                #
                # Unwind the integral term here rather than freezing it. Standing on the
                # brake, heading error is not a tracking error, and the trim that was
                # right at 1.2 m/s is not the trim for setting off again -- carrying it
                # over would put the hull into a turn the moment it releases the brake.
                self.steering_integral *= 0.98
                self.steering_command = approach(self.steering_command, 0.0, steering_delta)
                self.publish_command(self.steering_command, 0.0, 1.0)
                return

            # A gate that stays shut while the builder is trying to creep means the hull is
            # not translating -- the thing that used to become a pivot. Report it once per
            # episode so it is visible in the log rather than only in the trajectory.
            if speed_gate < 0.5 and not self.reported_speed_gate:
                self.reported_speed_gate = True
                self.get_logger().warn(
                    f"creeping at {abs(self.state.speed):.3f} m/s, under the "
                    f"{knee:.2f} m/s knee: steering gated to {speed_gate:.2f} of demand so "
                    f"the hull drives straight instead of pivoting."
                )
            elif speed_gate >= 0.999 and self.reported_speed_gate:
                self.reported_speed_gate = False
            keep_speed = max(0.0, float(self.get_parameter("station_keep_speed_mps").value))
            gain = max(0.0, float(self.get_parameter("station_keep_gain").value))
            min_speed = max(0.0, float(self.get_parameter("station_keep_min_speed_mps").value))
            # Floored, not tapered to zero -- see station_keep_min_speed_mps. A speed the
            # vehicle cannot make is the same as no command at all, and the builder simply
            # stops just outside the band and never signals that it is on station.
            target_speed = clamp(gain * arc_error, min(min_speed, keep_speed), keep_speed)
            speed_error = target_speed - self.state.speed
            kp = max(0.0, float(self.get_parameter("speed_kp").value))
            max_throttle = clamp(float(self.get_parameter("max_throttle").value), 0.0, 1.0)
            effort = kp * speed_error
            steer_cap = self.station_keep_steer_cap(steering_ff)
            self.publish_command(
                clamp(self.steering_command, -steer_cap, steer_cap),
                clamp(effort, 0.0, max_throttle),
                clamp(-effort, 0.0, 1.0),
            )
            return

        target_speed = max(
            0.0, float(self.get_parameter("target_speed_mps").value)
        )
        speed_error = target_speed - self.state.speed
        tolerance = max(
            0.0,
            float(self.get_parameter("speed_tolerance_mps").value),
        )
        kp = max(0.0, float(self.get_parameter("speed_kp").value))
        max_throttle = clamp(
            float(self.get_parameter("max_throttle").value), 0.0, 1.0
        )
        if abs(speed_error) <= tolerance:
            throttle = 0.0
            braking = 0.0
        else:
            effort = kp * speed_error
            throttle = clamp(effort, 0.0, max_throttle)
            braking = clamp(-effort, 0.0, 1.0)

        self.publish_command(
            self.steering_command, throttle, braking
        )

    def publish_command(
        self, steering: float, throttle: float, braking: float
    ) -> None:
        msg = Float64MultiArray()
        msg.data = [
            clamp(steering, -1.0, 1.0),
            clamp(throttle, 0.0, 1.0),
            clamp(braking, 0.0, 1.0),
        ]
        self.command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BuilderOrbitController()
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
