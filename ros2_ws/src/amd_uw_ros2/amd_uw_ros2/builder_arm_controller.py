import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class BuilderArmController(Node):
    """Command one safe, visible pose on a tracked builder arm."""

    def __init__(self) -> None:
        super().__init__("builder_arm_controller")

        self.declare_parameter("builder_id", 1)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("theta1", -math.pi)
        self.declare_parameter("theta2", 0.35)
        self.declare_parameter("theta3", 0.40)
        self.declare_parameter("theta4", -0.20)
        self.declare_parameter("finger_closure_m", 0.05)
        self.declare_parameter("joint_tolerance_rad", 0.03)

        self.builder_id = int(self.get_parameter("builder_id").value)
        self.command_topic = f"/builder_{self.builder_id}/arm_cmd"
        self.state_topic = f"/builder_{self.builder_id}/arm_state"
        self.last_state: Optional[list[float]] = None
        self.reported_reached = False
        self.reported_waiting = False

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
            f"builder_{self.builder_id} arm controller: "
            f"{self.command_topic} -> {self.state_topic}"
        )

    def target(self) -> list[float]:
        return [
            float(self.get_parameter("theta1").value),
            float(self.get_parameter("theta2").value),
            float(self.get_parameter("theta3").value),
            float(self.get_parameter("theta4").value),
            min(
                0.05,
                max(
                    0.0,
                    float(
                        self.get_parameter("finger_closure_m").value
                    ),
                ),
            ),
        ]

    def on_state(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 9:
            self.get_logger().warn(
                "Ignoring arm_state; expected 9 measured values."
            )
            return
        self.last_state = [float(value) for value in msg.data]

    def on_timer(self) -> None:
        if self.command_pub.get_subscription_count() == 0:
            if not self.reported_waiting:
                self.get_logger().warn(
                    f"Waiting for the simulation subscriber on "
                    f"{self.command_topic}."
                )
                self.reported_waiting = True
            return

        command = Float64MultiArray()
        command.data = self.target()
        self.command_pub.publish(command)

        if self.last_state is None or self.reported_reached:
            return

        tolerance = max(
            0.0,
            float(self.get_parameter("joint_tolerance_rad").value),
        )
        max_error = max(
            abs(actual - target)
            for actual, target in zip(self.last_state[:4], command.data[:4])
        )
        if max_error <= tolerance:
            self.reported_reached = True
            self.get_logger().info(
                "Builder arm reached target: "
                f"theta=({command.data[0]:.3f}, {command.data[1]:.3f}, "
                f"{command.data[2]:.3f}, {command.data[3]:.3f}), "
                f"max_error={max_error:.4f} rad."
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BuilderArmController()
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
