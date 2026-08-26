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
        self.declare_parameter("station_tolerance_rad", 0.05)
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
        self.declare_parameter("station_keep_deadband_m", 0.25)
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
        self.declare_parameter("station_keep_max_steering", 0.35)

        self.builder_id = int(self.get_parameter("builder_id").value)
        self.state_topic = f"/builder_{self.builder_id}/vehicle_state"
        self.command_topic = f"/builder_{self.builder_id}/vehicle_cmd"
        self.station_topic = f"/builder_{self.builder_id}/station_angle"
        self.arm_status_topic = f"/builder_{self.builder_id}/arm_status"
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
            self.station_angle = angle
            if self.holding_station:
                self.holding_station = False
                self.get_logger().info(
                    f"station moved to {math.degrees(angle):.1f} deg; driving to the new one."
                )

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
        raw = steering_ff + kp * head_err + ki * self.steering_integral
        # Anti-windup against the ACTUAL limit, not against 1.0 -- the output saturates at
        # `limit`, so that is where accumulating has to stop. The reversal exception lets a
        # saturated builder still wind the integral back down; without it the integral that
        # saturated it stays frozen at the value that keeps it there.
        if abs(raw) < limit or (raw > 0.0) != (head_err > 0.0):
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
        radius_tol = abs(float(self.get_parameter("station_radius_tol_m").value))
        radius_release = max(radius_tol, abs(float(self.get_parameter("station_radius_release_m").value)))
        # Two bands, not one: what it takes to TAKE station, and what it takes to give one
        # up. See station_radius_release_m.
        on_lane = abs(radius_error) <= radius_tol
        stay_on_lane = abs(radius_error) <= radius_release
        if not stay_on_lane and not self.reported_off_lane:
            self.reported_off_lane = True
            self.get_logger().warn(
                f"{radius_error:+.2f} m off the {radius:.1f} m lane; driving the lane back "
                f"before taking station."
            )
        elif stay_on_lane and self.reported_off_lane:
            self.reported_off_lane = False
            self.get_logger().info(f"back on the lane ({radius_error:+.2f} m).")

        if error is not None:
            tolerance = abs(float(self.get_parameter("station_tolerance_rad").value))
            release = max(tolerance, abs(float(self.get_parameter("station_release_rad").value)))
            # Arrival is judged on ABSOLUTE angular proximity, not on remaining
            # travel. Remaining travel is forced into [0, 2*pi), so overshooting
            # the station by one control tick makes it read ~2*pi -- "nearly a
            # full lap to go" -- and the builder commits to another whole orbit
            # instead of stopping the few centimetres past where it wanted to be.
            # Approaching from either side counts as being on station.
            if not self.holding_station and abs(wrap_to_pi(error)) <= tolerance and on_lane:
                self.holding_station = True
                self.get_logger().info(
                    f"holding station at {math.degrees(self.station_angle):.1f} deg."
                )
            elif self.holding_station and (abs(wrap_to_pi(error)) > release or not stay_on_lane):
                self.holding_station = False
                why = (
                    f"pushed {math.degrees(abs(wrap_to_pi(error))):.1f} deg off station, past the "
                    f"{math.degrees(release):.1f} deg release band"
                    if abs(wrap_to_pi(error)) > release
                    else f"pushed {radius_error:+.2f} m off the lane, past the "
                    f"{radius_release:.2f} m release band"
                )
                self.get_logger().info(f"{why}; driving round to re-acquire.")

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
            deadband = max(0.0, float(self.get_parameter("station_keep_deadband_m").value))
            if arc_error <= deadband:
                # On the mark, or crept slightly PAST it. Do not chase forward past
                # the station -- the builder only drives one way round, so overshoot
                # would cost a full lap. Sit on the brake.
                #
                # Unwind the integral term here rather than freezing it. Standing on the
                # brake, heading error is not a tracking error, and the trim that was
                # right at 1.2 m/s is not the trim for setting off again -- carrying it
                # over would put the hull into a turn the moment it releases the brake.
                self.steering_integral *= 0.98
                self.steering_command = approach(self.steering_command, 0.0, steering_delta)
                self.publish_command(self.steering_command, 0.0, 1.0)
                return

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
            steer_cap = abs(float(self.get_parameter("station_keep_max_steering").value))
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
