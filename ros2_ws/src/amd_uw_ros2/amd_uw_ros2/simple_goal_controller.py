import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


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


class SimpleGoalController(Node):
    """POC controller that drives one simulated robot toward a target point."""

    def __init__(self) -> None:
        super().__init__("simple_goal_controller")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("target_x", 5.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_speed_mps", 1.0)
        self.declare_parameter("stop_radius_m", 0.75)
        self.declare_parameter("slow_radius_m", 3.0)
        self.declare_parameter("heading_kp", 1.5)
        self.declare_parameter("speed_kp", 0.6)
        self.declare_parameter("max_target_speed_mps", 2.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.state_topic = f"/robot_{self.robot_id}/state"
        self.target_topic = f"/robot_{self.robot_id}/target_pose"
        self.command_topic = f"/robot_{self.robot_id}/vehicle_cmd"

        self.state: Optional[RobotState] = None
        self.target = Pose2D()
        self.target.x = float(self.get_parameter("target_x").value)
        self.target.y = float(self.get_parameter("target_y").value)
        self.target.theta = 0.0

        self.command_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.create_subscription(Float64MultiArray, self.state_topic, self.on_state, 10)
        self.create_subscription(Pose2D, self.target_topic, self.on_target, 10)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f"robot_{self.robot_id} controller: {self.state_topic} + {self.target_topic} -> {self.command_topic}"
        )

    def on_state(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warn("Ignoring state message; expected [x, y, yaw, speed].")
            return

        self.state = RobotState(
            x=float(msg.data[0]),
            y=float(msg.data[1]),
            yaw=float(msg.data[2]),
            speed=float(msg.data[3]),
        )

    def on_target(self, msg: Pose2D) -> None:
        self.target = msg
        self.get_logger().info(f"New target: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")

    def on_timer(self) -> None:
        if self.state is None:
            self.publish_command(VehicleCommand())
            return

        dx = self.target.x - self.state.x
        dy = self.target.y - self.state.y
        distance = math.hypot(dx, dy)

        if distance <= float(self.get_parameter("stop_radius_m").value):
            self.publish_command(VehicleCommand(steering=0.0, throttle=0.0, brake=1.0))
            return

        desired_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(desired_heading - self.state.yaw)
        steering = clamp(float(self.get_parameter("heading_kp").value) * heading_error, -1.0, 1.0)

        target_speed = min(
            float(self.get_parameter("target_speed_mps").value),
            float(self.get_parameter("max_target_speed_mps").value),
        )
        slow_radius = float(self.get_parameter("slow_radius_m").value)
        if distance < slow_radius:
            target_speed *= max(0.2, distance / slow_radius)

        speed_error = target_speed - self.state.speed
        effort = float(self.get_parameter("speed_kp").value) * speed_error

        throttle = clamp(effort, 0.0, 1.0)
        brake = clamp(-effort, 0.0, 1.0)

        self.publish_command(VehicleCommand(steering=steering, throttle=throttle, brake=brake))

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
    node = SimpleGoalController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
