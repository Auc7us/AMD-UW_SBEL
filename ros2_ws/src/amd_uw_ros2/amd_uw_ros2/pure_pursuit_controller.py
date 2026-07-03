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

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("target_speed_mps", 1.0)
        self.declare_parameter("speed_kp", 0.55)
        self.declare_parameter("speed_tolerance_mps", 0.08)
        self.declare_parameter("target_speed_ramp_mps2", 10.0)
        self.declare_parameter("throttle_ramp_per_s", 10.0)
        self.declare_parameter("brake_ramp_per_s", 10.0)
        self.declare_parameter("switch_radius_m", 1.0)
        self.declare_parameter("lookahead_min_m", 2.0)
        self.declare_parameter("wheelbase_m", 2.5)
        self.declare_parameter("max_steering_angle_rad", 0.6)
        self.declare_parameter("rock_side_offset_m", 1.5)
        self.declare_parameter("rear_reference_offset_m", 1.25)
        self.declare_parameter("pickup_angle_min_deg", 60.0)
        self.declare_parameter("pickup_angle_max_deg", 100.0)
        self.declare_parameter("pickup_slowdown_offset_m", 10.0)
        # Crawl into the pickup zone so a real full stop is reachable: on low-traction
        # lunar terrain a rover entering at 2 m/s just slides with the wheels locked
        # and never settles. Ramp down to a slow boundary speed and allow it to reach
        # ~0 at the boundary (min approach floor = 0).
        self.declare_parameter("pickup_min_approach_speed_mps", 0.0)
        self.declare_parameter("pickup_boundary_speed_mps", 1.0)
        self.declare_parameter("pickup_request_rate_hz", 1.0)
        # Only request a pickup once the chassis has actually come to a FULL STOP:
        # the arm IK is solved for the base pose at request time, and the rover
        # coasts after the wheels brake (low lunar traction), so requesting while
        # still moving leaves the gripper short of the rock. "Full stop" = speed at
        # or below stop_speed held continuously for stop_dwell_s (so we fire at rest,
        # not while still decelerating through the threshold). stop_timeout_s is a
        # safety fallback so a rover that can't fully settle still proceeds.
        self.declare_parameter("pickup_stop_speed_mps", 0.05)
        self.declare_parameter("pickup_stop_dwell_s", 0.4)
        self.declare_parameter("pickup_stop_timeout_s", 10.0)
        self.declare_parameter("post_done_straighten_time_s", 0.75)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.ego_state_topic = f"/robot_{self.robot_id}/egoState"
        self.target_pos_topic = f"/robot_{self.robot_id}/targetPos"
        self.target_done_topic = f"/robot_{self.robot_id}/target_done"
        self.pickup_request_topic = f"/robot_{self.robot_id}/pickup_request"
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
        self.command = VehicleCommand()
        self.ramped_target_speed = 0.0

        self.command_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.pickup_request_pub = self.create_publisher(Float64MultiArray, self.pickup_request_topic, 10)
        self.create_subscription(Float64MultiArray, self.ego_state_topic, self.on_ego_state, 10)
        self.create_subscription(Float64MultiArray, self.target_pos_topic, self.on_target_pos, 10)
        self.create_subscription(Bool, self.target_done_topic, self.on_target_done, 10)

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

        self.targets = targets
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
        self.straighten_until_time_s = (
            self.now_seconds() + max(0.0, float(self.get_parameter("post_done_straighten_time_s").value))
        )

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
        if self.state is None or not self.targets:
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
                waited = self.now_seconds() - (self.wait_start_s or self.now_seconds())
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

            if not self.is_target_in_pickup_position(switch_radius):
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
            self.wait_start_s = self.now_seconds()
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
        lookahead = max(distance, float(self.get_parameter("lookahead_min_m").value))
        wheelbase = max(1e-6, float(self.get_parameter("wheelbase_m").value))
        max_angle = max(1e-6, float(self.get_parameter("max_steering_angle_rad").value))

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
        return VehicleCommand(
            steering=0.0,
            throttle=clamp(effort, 0.0, 1.0),
            brake=clamp(-effort, 0.0, 1.0),
        )

    def ramp_command(self, target: VehicleCommand) -> VehicleCommand:
        throttle_delta = max(0.0, float(self.get_parameter("throttle_ramp_per_s").value)) * self.dt
        brake_delta = max(0.0, float(self.get_parameter("brake_ramp_per_s").value)) * self.dt

        return VehicleCommand(
            steering=target.steering,
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

    def is_target_in_pickup_position(self, switch_radius: float) -> bool:
        target_x, target_y = self.get_drive_target()
        ref_x, ref_y = self.rear_reference_position()
        target_distance = math.hypot(target_x - ref_x, target_y - ref_y)
        return target_distance <= switch_radius and self.rock_is_in_pickup_angle_sector()

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

    def is_straightening_after_done(self) -> bool:
        if self.straighten_until_time_s is None:
            return False

        if self.now_seconds() < self.straighten_until_time_s:
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
