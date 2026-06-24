import math
from dataclasses import dataclass
from typing import List, Optional, Set, Tuple

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
        self.declare_parameter("target_speed_ramp_mps2", 0.35)
        self.declare_parameter("throttle_ramp_per_s", 0.7)
        self.declare_parameter("brake_ramp_per_s", 1.2)
        self.declare_parameter("switch_radius_m", 1.0)
        self.declare_parameter("lookahead_min_m", 2.0)
        self.declare_parameter("wheelbase_m", 2.5)
        self.declare_parameter("max_steering_angle_rad", 0.6)
        self.declare_parameter("rock_side_offset_m", 2.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.ego_state_topic = f"/robot_{self.robot_id}/egoState"
        self.target_pos_topic = f"/robot_{self.robot_id}/targetPos"
        self.command_topic = f"/robot_{self.robot_id}/vehicle_cmd"

        self.state: Optional[RobotState] = None
        self.targets: List[Tuple[float, float]] = []
        self.target_index = -1
        self.completed_targets: Set[int] = set()
        self.drive_target_index = -1
        self.drive_target_offset = (0.0, 0.0)
        self.have_targets = False
        self.command = VehicleCommand()
        self.ramped_target_speed = 0.0

        self.command_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.create_subscription(Float64MultiArray, self.ego_state_topic, self.on_ego_state, 10)
        self.create_subscription(Float64MultiArray, self.target_pos_topic, self.on_target_pos, 10)

        rate_hz = max(1e-6, float(self.get_parameter("control_rate_hz").value))
        self.dt = 1.0 / rate_hz
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"robot_{self.robot_id} pure pursuit: {self.ego_state_topic} + "
            f"{self.target_pos_topic} -> {self.command_topic}"
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
        if len(msg.data) % 2 != 0:
            self.get_logger().warn("Ignoring targetPos message; expected [x0, y0, x1, y1, ...].")
            return

        targets = [
            (float(msg.data[i]), float(msg.data[i + 1]))
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

    def on_timer(self) -> None:
        if self.state is None or not self.targets:
            self.command = self.ramp_command(VehicleCommand())
            self.publish_command(self.command)
            return

        switch_radius = max(0.0, float(self.get_parameter("switch_radius_m").value))
        while True:
            if not self.ensure_active_target():
                self.command = self.ramp_command(VehicleCommand())
                self.publish_command(self.command)
                return

            target_x, target_y = self.get_drive_target()
            if math.hypot(target_x - self.state.x, target_y - self.state.y) > switch_radius:
                break
            self.get_logger().info(f"Reached targetPos[{self.target_index}], switching to next target.")
            self.completed_targets.add(self.target_index)
            self.target_index = -1
            self.drive_target_index = -1

        target = self.get_drive_target()
        steering = self.compute_steering(target)
        speed_command = self.compute_speed_command()
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
        target_x, target_y = self.targets[self.target_index]
        distance = math.hypot(target_x - self.state.x, target_y - self.state.y)
        self.get_logger().info(
            f"Selected nearest targetPos[{self.target_index}] at ({target_x:.2f}, {target_y:.2f}), "
            f"distance={distance:.2f} m."
        )
        return True

    def get_drive_target(self) -> Tuple[float, float]:
        rock_x, rock_y = self.targets[self.target_index]
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

    def compute_speed_command(self) -> VehicleCommand:
        target_speed = max(0.0, float(self.get_parameter("target_speed_mps").value))
        if target_speed >= self.state.speed and self.ramped_target_speed < self.state.speed:
            self.ramped_target_speed = self.state.speed

        target_ramp = max(0.0, float(self.get_parameter("target_speed_ramp_mps2").value)) * self.dt
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
