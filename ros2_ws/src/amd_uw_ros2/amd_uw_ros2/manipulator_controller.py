from dataclasses import dataclass
import time
from typing import Optional, Set

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


ARM_STATE_IDLE = 0
ARM_STATE_BUSY = 1
ARM_STATE_DONE = 2
ARM_STATE_FAILED = 3


@dataclass
class PickupRequest:
    target_index: int
    rock_x: float
    rock_y: float


@dataclass
class ActiveCommand:
    command_seq: int
    target_index: int
    rock_x: float
    rock_y: float
    last_publish_time_s: float
    status_seen: bool = False


class ManipulatorController(Node):
    """Coordinate high-level pickup requests with the C++ Chrono arm executor."""

    def __init__(self) -> None:
        super().__init__("manipulator_controller")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("skip_failed_targets", True)
        self.declare_parameter("command_timeout_s", 120.0)
        self.declare_parameter("arm_cmd_republish_rate_hz", 1.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.pickup_request_topic = f"/robot_{self.robot_id}/pickup_request"
        self.arm_cmd_topic = f"/robot_{self.robot_id}/arm_cmd"
        self.arm_status_topic = f"/robot_{self.robot_id}/arm_status"
        self.target_done_topic = f"/robot_{self.robot_id}/target_done"

        # Chrono ignores arm commands with old sequence numbers. Seed from a
        # monotonic clock so restarting this ROS node does not replay seq=1.
        self.command_seq = int(time.monotonic_ns() // 1_000_000)
        self.active: Optional[ActiveCommand] = None
        self.active_start_time_s: Optional[float] = None
        self.completed_targets: Set[int] = set()

        self.arm_cmd_pub = self.create_publisher(Float64MultiArray, self.arm_cmd_topic, 10)
        self.target_done_pub = self.create_publisher(Bool, self.target_done_topic, 10)
        self.create_subscription(Float64MultiArray, self.pickup_request_topic, self.on_pickup_request, 10)
        self.create_subscription(Float64MultiArray, self.arm_status_topic, self.on_arm_status, 10)
        self.timer = self.create_timer(0.5, self.on_timer)

        self.get_logger().info(
            f"robot_{self.robot_id} manipulator: {self.pickup_request_topic} -> "
            f"{self.arm_cmd_topic}; {self.arm_status_topic} -> {self.target_done_topic}"
        )

    def on_pickup_request(self, msg: Float64MultiArray) -> None:
        request = self.parse_pickup_request(msg)
        if request is None:
            return

        if request.target_index in self.completed_targets:
            return

        if self.active is not None:
            if self.active.target_index != request.target_index:
                self.get_logger().warn(
                    f"Ignoring pickup_request for target {request.target_index}; "
                    f"target {self.active.target_index} is already active."
                )
            return

        self.command_seq += 1
        now_s = self.now_seconds()
        self.active = ActiveCommand(
            self.command_seq,
            request.target_index,
            request.rock_x,
            request.rock_y,
            now_s,
        )
        self.active_start_time_s = now_s

        self.publish_active_command()
        self.get_logger().info(
            f"Sent arm_cmd seq={self.command_seq} target={request.target_index} "
            f"rock=({request.rock_x:.2f}, {request.rock_y:.2f})."
        )

    def on_arm_status(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 5:
            self.get_logger().warn("Ignoring arm_status; expected [command_seq, state, target_index, success, error_code].")
            return

        command_seq = int(round(float(msg.data[0])))
        state = int(round(float(msg.data[1])))
        target_index = int(round(float(msg.data[2])))
        success = bool(round(float(msg.data[3])))
        error_code = int(round(float(msg.data[4])))

        if self.active is None or command_seq != self.active.command_seq:
            return

        self.active.status_seen = True

        if state == ARM_STATE_DONE and success:
            self.publish_target_done(target_index)
            self.get_logger().info(f"Manipulator completed target {target_index}; published target_done=true.")
            self.active = None
            self.active_start_time_s = None
        elif state == ARM_STATE_FAILED:
            self.get_logger().warn(
                f"Manipulator failed target {target_index} with error_code={error_code}."
            )
            if bool(self.get_parameter("skip_failed_targets").value):
                self.publish_target_done(target_index)
                self.get_logger().warn(f"Skipping failed target {target_index}; published target_done=true.")
            self.active = None
            self.active_start_time_s = None

    def on_timer(self) -> None:
        if self.active is None or self.active_start_time_s is None:
            return

        self.republish_active_command_if_needed()

        timeout = max(0.0, float(self.get_parameter("command_timeout_s").value))
        if timeout <= 0.0:
            return

        if self.now_seconds() - self.active_start_time_s > timeout:
            target_index = self.active.target_index
            self.get_logger().warn(f"Manipulator command timed out for target {target_index}.")
            if bool(self.get_parameter("skip_failed_targets").value):
                self.publish_target_done(target_index)
                self.get_logger().warn(f"Skipping timed-out target {target_index}; published target_done=true.")
            self.active = None
            self.active_start_time_s = None

    def publish_active_command(self) -> None:
        if self.active is None:
            return

        cmd = Float64MultiArray()
        cmd.data = [
            float(self.active.command_seq),
            float(self.active.target_index),
            self.active.rock_x,
            self.active.rock_y,
        ]
        self.arm_cmd_pub.publish(cmd)
        self.active.last_publish_time_s = self.now_seconds()

    def republish_active_command_if_needed(self) -> None:
        if self.active is None or self.active.status_seen:
            return

        rate_hz = max(0.0, float(self.get_parameter("arm_cmd_republish_rate_hz").value))
        if rate_hz <= 0.0:
            return

        if self.now_seconds() - self.active.last_publish_time_s >= 1.0 / rate_hz:
            self.publish_active_command()

    def publish_target_done(self, target_index: int) -> None:
        self.completed_targets.add(target_index)
        msg = Bool()
        msg.data = True
        self.target_done_pub.publish(msg)

    def parse_pickup_request(self, msg: Float64MultiArray) -> Optional[PickupRequest]:
        if len(msg.data) < 3:
            self.get_logger().warn("Ignoring pickup_request; expected [target_index, rock_x_global, rock_y_global].")
            return None

        return PickupRequest(
            target_index=int(round(float(msg.data[0]))),
            rock_x=float(msg.data[1]),
            rock_y=float(msg.data[2]),
        )

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulatorController()
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
