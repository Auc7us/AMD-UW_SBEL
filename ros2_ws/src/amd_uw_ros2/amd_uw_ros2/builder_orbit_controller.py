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


def circle_lookahead_target(
    state: BuilderState,
    center_x: float,
    center_y: float,
    radius: float,
    lookahead_m: float,
    counter_clockwise: bool,
) -> Tuple[float, float]:
    """Return a point one lookahead arc ahead on the requested circle."""
    radial_angle = math.atan2(state.y - center_y, state.x - center_x)
    direction = 1.0 if counter_clockwise else -1.0
    target_angle = radial_angle + direction * lookahead_m / radius
    return (
        center_x + radius * math.cos(target_angle),
        center_y + radius * math.sin(target_angle),
    )


def circle_pure_pursuit_steering(
    state: BuilderState,
    center_x: float,
    center_y: float,
    radius: float,
    lookahead_m: float,
    curvature_to_steering: float,
    counter_clockwise: bool,
) -> float:
    """Map pure-pursuit curvature to the M113 normalized steering input."""
    target_x, target_y = circle_lookahead_target(
        state,
        center_x,
        center_y,
        radius,
        lookahead_m,
        counter_clockwise,
    )
    dx = target_x - state.x
    dy = target_y - state.y
    local_y = -math.sin(state.yaw) * dx + math.cos(state.yaw) * dy
    chord_sq = dx * dx + dy * dy
    if chord_sq < 1e-9:
        return 0.0
    curvature = 2.0 * local_y / chord_sq
    return clamp(curvature_to_steering * curvature, -1.0, 1.0)


class BuilderOrbitController(Node):
    """Drive one tracked builder around a concentric circular path."""

    def __init__(self) -> None:
        super().__init__("builder_orbit_controller")

        self.declare_parameter("builder_id", 1)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("work_circle_radius_m", 30.0)
        self.declare_parameter("path_radius_m", 35.0)
        self.declare_parameter("counter_clockwise", True)
        self.declare_parameter("lookahead_m", 8.0)
        self.declare_parameter("curvature_to_steering", 4.0)
        self.declare_parameter("steering_ramp_per_s", 0.5)
        self.declare_parameter("target_speed_mps", 1.0)
        self.declare_parameter("speed_kp", 0.6)
        self.declare_parameter("speed_tolerance_mps", 0.05)
        self.declare_parameter("max_throttle", 0.6)
        # How close to the station angle counts as arrived, and how much further it
        # must drift before setting off again. The gap between them is hysteresis: a
        # tracked vehicle cannot stop on a point, so without it the builder would
        # creep past, restart, and hunt around its station forever.
        self.declare_parameter("station_tolerance_rad", 0.05)
        self.declare_parameter("station_release_rad", 0.20)

        self.builder_id = int(self.get_parameter("builder_id").value)
        self.state_topic = f"/builder_{self.builder_id}/vehicle_state"
        self.command_topic = f"/builder_{self.builder_id}/vehicle_cmd"
        self.station_topic = f"/builder_{self.builder_id}/station_angle"
        self.state: Optional[BuilderState] = None
        self.steering_command = 0.0
        # Angle on the orbit this builder should wait at, published by the sim. It
        # steps one harvest cycle each time this rank's collector dumps a load, so the
        # builder stays radially inboard of the CURRENT drop point instead of circling
        # forever past a pile that has moved on.
        self.station_angle: Optional[float] = None
        self.holding_station = False

        self.command_pub = self.create_publisher(
            Float64MultiArray, self.command_topic, 10
        )
        self.create_subscription(
            Float64MultiArray, self.state_topic, self.on_state, 10
        )
        self.create_subscription(
            Float64MultiArray, self.station_topic, self.on_station_angle, 10
        )

        rate_hz = max(
            1.0, float(self.get_parameter("control_rate_hz").value)
        )
        self.dt = 1.0 / rate_hz
        self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"builder_{self.builder_id} orbit controller: "
            f"work radius={self.get_parameter('work_circle_radius_m').value:.1f} m, "
            f"path radius={self.get_parameter('path_radius_m').value:.1f} m, "
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

        radius = max(
            1e-3, float(self.get_parameter("path_radius_m").value)
        )
        lookahead = max(
            1e-3, float(self.get_parameter("lookahead_m").value)
        )
        target_steering = circle_pure_pursuit_steering(
            self.state,
            float(self.get_parameter("center_x").value),
            float(self.get_parameter("center_y").value),
            radius,
            lookahead,
            float(
                self.get_parameter("curvature_to_steering").value
            ),
            bool(self.get_parameter("counter_clockwise").value),
        )
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
        error = self.station_error()
        if error is not None:
            tolerance = abs(float(self.get_parameter("station_tolerance_rad").value))
            release = max(tolerance, abs(float(self.get_parameter("station_release_rad").value)))
            # Arrival is judged on ABSOLUTE angular proximity, not on remaining
            # travel. Remaining travel is forced into [0, 2*pi), so overshooting
            # the station by one control tick makes it read ~2*pi -- "nearly a
            # full lap to go" -- and the builder commits to another whole orbit
            # instead of stopping the few centimetres past where it wanted to be.
            # Approaching from either side counts as being on station.
            if not self.holding_station and abs(wrap_to_pi(error)) <= tolerance:
                self.holding_station = True
                self.get_logger().info(
                    f"holding station at {math.degrees(self.station_angle):.1f} deg."
                )
            elif self.holding_station and abs(wrap_to_pi(error)) > release:
                self.holding_station = False
                self.get_logger().info("drifted off station; re-acquiring.")

        if self.holding_station:
            self.steering_command = approach(self.steering_command, 0.0, steering_delta)
            self.publish_command(self.steering_command, 0.0, 1.0)
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
