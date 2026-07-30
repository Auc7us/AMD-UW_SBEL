from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class BuilderDriveController(Node):
    """Basic closed-loop speed controller for one tracked builder."""

    def __init__(self) -> None:
        super().__init__("builder_drive_controller")

        self.declare_parameter("builder_id", 1)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("target_speed_mps", 0.8)
        self.declare_parameter("steering", 0.15)
        self.declare_parameter("speed_kp", 0.6)
        self.declare_parameter("max_throttle", 0.5)

        self.builder_id = int(self.get_parameter("builder_id").value)
        self.command_topic = f"/builder_{self.builder_id}/vehicle_cmd"
        self.state_topic = f"/builder_{self.builder_id}/vehicle_state"
        self.speed: Optional[float] = None

        self.command_pub = self.create_publisher(
            Float64MultiArray, self.command_topic, 10
        )
        self.create_subscription(
            Float64MultiArray, self.state_topic, self.on_state, 10
        )

        rate_hz = max(
            1.0, float(self.get_parameter("control_rate_hz").value)
        )
        self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(
            f"builder_{self.builder_id} drive controller: "
            f"{self.state_topic} -> {self.command_topic}"
        )

    def on_state(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().warn(
                "Ignoring vehicle_state; expected [x, y, yaw, speed]."
            )
            return
        self.speed = float(msg.data[3])

    def on_timer(self) -> None:
        msg = Float64MultiArray()
        if self.speed is None:
            msg.data = [0.0, 0.0, 1.0]
            self.command_pub.publish(msg)
            return

        target_speed = max(
            0.0, float(self.get_parameter("target_speed_mps").value)
        )
        steering = clamp(
            float(self.get_parameter("steering").value), -1.0, 1.0
        )
        kp = max(0.0, float(self.get_parameter("speed_kp").value))
        max_throttle = clamp(
            float(self.get_parameter("max_throttle").value), 0.0, 1.0
        )
        effort = kp * (target_speed - self.speed)
        msg.data = [
            steering,
            clamp(effort, 0.0, max_throttle),
            clamp(-effort, 0.0, 1.0),
        ]
        self.command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BuilderDriveController()
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
