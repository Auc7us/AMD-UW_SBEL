from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def approach(current: float, target: float, max_delta: float) -> float:
    if current < target:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


@dataclass
class RobotState:
    speed: float


@dataclass
class VehicleCommand:
    steering: float = 0.0
    throttle: float = 0.0
    brake: float = 1.0


class ConstantSpeedController(Node):
    """Speed-only POC controller with ramp-limited throttle and brake."""

    def __init__(self) -> None:
        super().__init__("constant_speed_controller")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("target_speed_mps", 1.0)
        self.declare_parameter("speed_kp", 0.55)
        self.declare_parameter("speed_tolerance_mps", 0.08)
        self.declare_parameter("target_speed_ramp_mps2", 0.35)
        self.declare_parameter("throttle_ramp_per_s", 0.7)
        self.declare_parameter("brake_ramp_per_s", 1.2)
        self.declare_parameter("coast_brake", 0.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.state_topic = f"/robot_{self.robot_id}/state"
        self.command_topic = f"/robot_{self.robot_id}/vehicle_cmd"

        self.state: Optional[RobotState] = None
        self.command = VehicleCommand()
        self.ramped_target_speed = 0.0

        self.command_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.create_subscription(Float64MultiArray, self.state_topic, self.on_state, 10)

        self.rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"robot_{self.robot_id} constant speed controller: {self.state_topic} -> {self.command_topic}"
        )

    def on_state(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warn("Ignoring state message; expected [x, y, yaw, speed].")
            return

        self.state = RobotState(speed=float(msg.data[3]))

    def on_timer(self) -> None:
        target_speed = max(0.0, float(self.get_parameter("target_speed_mps").value))
        target_ramp = max(0.0, float(self.get_parameter("target_speed_ramp_mps2").value)) * self.dt
        self.ramped_target_speed = approach(self.ramped_target_speed, target_speed, target_ramp)

        if self.state is None:
            self.command = self.ramp_command(VehicleCommand(steering=0.0, throttle=0.0, brake=1.0))
            self.publish_command(self.command)
            return

        speed_error = self.ramped_target_speed - self.state.speed
        tolerance = max(0.0, float(self.get_parameter("speed_tolerance_mps").value))
        speed_kp = max(0.0, float(self.get_parameter("speed_kp").value))
        coast_brake = clamp(float(self.get_parameter("coast_brake").value), 0.0, 1.0)

        if abs(speed_error) <= tolerance:
            target_command = VehicleCommand(steering=0.0, throttle=0.0, brake=coast_brake)
        else:
            effort = speed_kp * speed_error
            target_command = VehicleCommand(
                steering=0.0,
                throttle=clamp(effort, 0.0, 1.0),
                brake=clamp(-effort, 0.0, 1.0),
            )

        self.command = self.ramp_command(target_command)
        self.publish_command(self.command)

    def ramp_command(self, target: VehicleCommand) -> VehicleCommand:
        throttle_delta = max(0.0, float(self.get_parameter("throttle_ramp_per_s").value)) * self.dt
        brake_delta = max(0.0, float(self.get_parameter("brake_ramp_per_s").value)) * self.dt

        return VehicleCommand(
            steering=0.0,
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
    node = ConstantSpeedController()
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
