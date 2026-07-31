from dataclasses import dataclass
import math
import time
from typing import Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray


ARM_STATE_IDLE = 0
ARM_STATE_BUSY = 1
ARM_STATE_DONE = 2
ARM_STATE_FAILED = 3

GRAB_HEIGHT_M = 0.22

try:
    from .inverse_kinematics import RobotArmInverseKinematicsSolver
except Exception:
    RobotArmInverseKinematicsSolver = None


@dataclass
class PickupRequest:
    target_index: int
    rock_x: float
    rock_y: float
    rock_z: Optional[float] = None


@dataclass
class EgoState:
    x: float
    y: float
    yaw: float
    speed: float
    z: float = 0.5


@dataclass
class ArmBasePose:
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float


@dataclass
class ActiveCommand:
    command_seq: int
    target_index: int
    rock_x: float
    rock_y: float
    theta: Tuple[float, float, float, float]
    last_publish_time_s: float
    status_seen: bool = False
    place_theta: Optional[Tuple[float, float, float, float]] = None


class ManipulatorController(Node):
    """Coordinate high-level pickup requests with the C++ Chrono arm executor."""

    def __init__(self) -> None:
        super().__init__("manipulator_controller")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("skip_failed_targets", True)
        # Wall-clock BACKSTOP only (sim not progressing at all). Generous on purpose.
        self.declare_parameter("command_timeout_s", 1800.0)
        # The real deadline, on the sim clock reported by arm_status.
        self.declare_parameter("command_timeout_sim_s", 90.0)
        self.declare_parameter("arm_cmd_republish_rate_hz", 1.0)
        # Aim the gripper at the rock's actual center height (rock_z reported by the
        # sim) plus a tiny offset, instead of a fixed height above ground -- so the
        # grab tracks each rock's real size. The small offset also absorbs the ~4 cm
        # the gripper settles below the FK target, landing it near the rock center.
        self.declare_parameter("grab_z_offset_m", 0.05)
        # Fallback gripper height (above ground) only if the sim doesn't report rock_z.
        self.declare_parameter("grab_height_m", GRAB_HEIGHT_M)
        # Must match the physical LrvArm geometry scale. LRVs use 1.0; the M113
        # tracked-builder reference uses 2.0.
        self.declare_parameter("arm_scale", 1.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.pickup_request_topic = f"/robot_{self.robot_id}/pickup_request"
        self.ego_state_topic = f"/robot_{self.robot_id}/egoState"
        self.arm_base_pose_topic = f"/robot_{self.robot_id}/arm_base_pose"
        self.place_target_topic = f"/robot_{self.robot_id}/place_target"
        self.arm_cmd_topic = f"/robot_{self.robot_id}/arm_cmd"
        self.arm_status_topic = f"/robot_{self.robot_id}/arm_status"
        self.target_done_topic = f"/robot_{self.robot_id}/target_done"
        self.target_pos_topic = f"/robot_{self.robot_id}/targetPos"
        self.mission_done_topic = f"/robot_{self.robot_id}/mission_done"
        self.at_home_topic = f"/robot_{self.robot_id}/at_home"
        self.trailer_cmd_topic = f"/robot_{self.robot_id}/trailer_cmd"
        self.trailer_state_topic = f"/robot_{self.robot_id}/trailer_state"

        # Chrono ignores arm commands with old sequence numbers. Seed from a
        # monotonic clock so restarting this ROS node does not replay seq=1.
        self.command_seq = int(time.monotonic_ns() // 1_000_000)
        self.active: Optional[ActiveCommand] = None
        self.active_start_time_s: Optional[float] = None
        self.active_start_sim_s: Optional[float] = None
        self.sim_time: Optional[float] = None
        self.ego_state: Optional[EgoState] = None
        self.arm_base_pose: Optional[ArmBasePose] = None
        self.place_target: Optional[Tuple[float, float, float]] = None
        self.completed_targets: Set[int] = set()
        # End-of-mission sequencing: all targets resolved -> drive home -> dump.
        self.num_targets = 0
        # Set when mission completion is seen on the topic, from ANY source -- this
        # node's own detection, a supervisor, or a human. Gating on "did I publish
        # it myself" meant an externally declared mission never dumped.
        self.mission_done = False
        self.dump_requested = False
        self.dump_finished = False
        self.arm_scale = float(self.get_parameter("arm_scale").value)
        self.ik_solver = (
            RobotArmInverseKinematicsSolver(scale=self.arm_scale)
            if RobotArmInverseKinematicsSolver is not None
            else None
        )

        self.arm_cmd_pub = self.create_publisher(Float64MultiArray, self.arm_cmd_topic, 10)
        self.target_done_pub = self.create_publisher(Bool, self.target_done_topic, 10)
        self.create_subscription(Float64MultiArray, self.pickup_request_topic, self.on_pickup_request, 10)
        self.create_subscription(Float64MultiArray, self.ego_state_topic, self.on_ego_state, 10)
        self.create_subscription(Float64MultiArray, self.arm_base_pose_topic, self.on_arm_base_pose, 10)
        self.create_subscription(Float64MultiArray, self.place_target_topic, self.on_place_target, 10)
        self.create_subscription(Float64MultiArray, self.arm_status_topic, self.on_arm_status, 10)
        self.mission_done_pub = self.create_publisher(Bool, self.mission_done_topic, 10)
        self.trailer_cmd_pub = self.create_publisher(Float64MultiArray, self.trailer_cmd_topic, 10)
        self.create_subscription(Float64MultiArray, self.target_pos_topic, self.on_target_pos, 10)
        self.create_subscription(Bool, self.mission_done_topic, self.on_mission_done, 10)
        self.create_subscription(Bool, self.at_home_topic, self.on_at_home, 10)
        self.create_subscription(Float64MultiArray, self.trailer_state_topic, self.on_trailer_state, 10)
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

        theta = self.compute_grab_theta(request)
        if theta is None:
            return

        # Solve the drop pose with the same IK, in the same arm-base frame, from the
        # place target the C++ arm published. If unavailable/unreachable, leave it
        # None and the arm falls back to its own place IK.
        place_theta = self.compute_place_theta(request.target_index)

        self.command_seq += 1
        now_s = self.now_seconds()
        self.active = ActiveCommand(
            self.command_seq,
            request.target_index,
            request.rock_x,
            request.rock_y,
            theta,
            now_s,
            place_theta=place_theta,
        )
        self.active_start_time_s = now_s
        self.active_start_sim_s = self.sim_time

        self.publish_active_command()
        z_text = f", {request.rock_z:.2f}" if request.rock_z is not None else ""
        self.get_logger().info(
            f"Sent arm_cmd seq={self.command_seq} target={request.target_index} "
            f"rock=({request.rock_x:.2f}, {request.rock_y:.2f}{z_text}) "
            f"theta={[round(t, 3) for t in theta]} "
            f"place_theta={[round(t, 3) for t in place_theta] if place_theta else None}."
        )

    def on_ego_state(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warn("Ignoring egoState; expected [x, y, yaw, speed, optional z].")
            return
        self.ego_state = EgoState(
            x=float(msg.data[0]),
            y=float(msg.data[1]),
            yaw=float(msg.data[2]),
            speed=float(msg.data[3]),
            z=float(msg.data[4]) if len(msg.data) >= 5 else 0.5,
        )

    def on_arm_base_pose(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 7:
            self.get_logger().warn("Ignoring arm_base_pose; expected [x, y, z, qw, qx, qy, qz].")
            return
        self.arm_base_pose = ArmBasePose(
            x=float(msg.data[0]),
            y=float(msg.data[1]),
            z=float(msg.data[2]),
            qw=float(msg.data[3]),
            qx=float(msg.data[4]),
            qy=float(msg.data[5]),
            qz=float(msg.data[6]),
        )

    def on_place_target(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warn("Ignoring place_target; expected [x, y, z].")
            return
        self.place_target = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

    def on_arm_status(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 5:
            self.get_logger().warn("Ignoring arm_status; expected [command_seq, state, target_index, success, error_code].")
            return

        command_seq = int(round(float(msg.data[0])))
        state = int(round(float(msg.data[1])))
        target_index = int(round(float(msg.data[2])))
        success = bool(round(float(msg.data[3])))
        error_code = int(round(float(msg.data[4])))
        # Optional 6th element: the sim's own clock. The arm advances on sim time, so
        # that is the only clock its deadline can sensibly use.
        if len(msg.data) >= 6:
            self.sim_time = float(msg.data[5])

        if self.active is None or command_seq != self.active.command_seq:
            return

        self.active.status_seen = True
        # Anchor the deadline to the first sim time seen for THIS command, not to the
        # time the command was sent: the sim may not have picked it up yet.
        if self.active_start_sim_s is None and self.sim_time is not None:
            self.active_start_sim_s = self.sim_time

        if state == ARM_STATE_DONE and success:
            self.publish_target_done(target_index)
            self.get_logger().info(f"Manipulator completed target {target_index}; published target_done=true.")
            self.active = None
            self.active_start_time_s = None
            self.active_start_sim_s = None
        elif state == ARM_STATE_FAILED:
            self.get_logger().warn(
                f"Manipulator failed target {target_index} with error_code={error_code}."
            )
            if bool(self.get_parameter("skip_failed_targets").value):
                self.publish_target_done(target_index)
                self.get_logger().warn(f"Skipping failed target {target_index}; published target_done=true.")
            self.active = None
            self.active_start_time_s = None
            self.active_start_sim_s = None

    def on_timer(self) -> None:
        if self.active is None or self.active_start_time_s is None:
            return

        self.republish_active_command_if_needed()

        # Two independent deadlines, because they catch different failures.
        #
        # command_timeout_sim_s is the real one, measured on the SIM clock reported by
        # arm_status. A full pick-and-place takes ~10-20 s of sim time, and the sim runs
        # roughly 19x slower than wall -- so the old 120 s WALL budget was ~6 s of sim
        # and fired on essentially every successful grab, skipping the target as failed
        # while the arm was still working correctly. Worse, the wall/sim ratio moves with
        # rank count, terrain cost and machine, so no fixed wall number is ever right.
        #
        # command_timeout_s stays as a wall-clock backstop for the case the sim-time
        # clock never advances at all: sim dead, crashed, or never received the command.
        # That is a liveness check, not a duration budget, so it should be generous.
        target_index = self.active.target_index
        sim_timeout = max(0.0, float(self.get_parameter("command_timeout_sim_s").value))
        if sim_timeout > 0.0 and self.sim_time is not None and self.active_start_sim_s is not None:
            elapsed_sim = self.sim_time - self.active_start_sim_s
            if elapsed_sim > sim_timeout:
                self.fail_active_command(
                    f"Manipulator command timed out for target {target_index} after "
                    f"{elapsed_sim:.1f} s of SIM time (limit {sim_timeout:.1f})."
                )
                return

        wall_timeout = max(0.0, float(self.get_parameter("command_timeout_s").value))
        if wall_timeout <= 0.0:
            return
        if self.now_seconds() - self.active_start_time_s > wall_timeout:
            stalled = "no arm_status ever arrived" if not self.active.status_seen else (
                f"sim clock stuck at {self.sim_time}" if self.sim_time is not None else "no sim clock reported"
            )
            self.fail_active_command(
                f"Manipulator command for target {target_index} hit the {wall_timeout:.0f} s WALL backstop "
                f"({stalled}). This means the sim is not progressing, not that the arm is slow."
            )

    def fail_active_command(self, message: str) -> None:
        if self.active is None:
            return
        target_index = self.active.target_index
        self.get_logger().warn(message)
        if bool(self.get_parameter("skip_failed_targets").value):
            self.publish_target_done(target_index)
            self.get_logger().warn(f"Skipping target {target_index}; published target_done=true.")
        self.active = None
        self.active_start_time_s = None
        self.active_start_sim_s = None

    def publish_active_command(self) -> None:
        if self.active is None:
            return

        cmd = Float64MultiArray()
        cmd.data = [
            float(self.active.command_seq),
            float(self.active.target_index),
            self.active.rock_x,
            self.active.rock_y,
            *[float(theta) for theta in self.active.theta],
        ]
        # Append the Python-solved drop pose (12-element command) when available;
        # otherwise the 8-element command tells the arm to use its own place IK.
        if self.active.place_theta is not None:
            cmd.data.extend(float(theta) for theta in self.active.place_theta)
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

    def on_target_pos(self, msg: Float64MultiArray) -> None:
        """Track how many rocks exist, so 'all of them resolved' is detectable."""
        stride = 3 if len(msg.data) % 3 == 0 else 2
        count = len(msg.data) // stride
        # A GROWING list means the sim started a new harvest cycle: the lane rotated
        # and fresh rocks are out. The end-of-mission flags are one-shot latches, so
        # they must be cleared or this node would never declare mission_done again and
        # never request another dump -- the rover would collect the new rocks and then
        # sit there holding them. Indices are stable (the sim appends), so
        # completed_targets is deliberately left alone.
        if count > self.num_targets and self.num_targets > 0:
            self.get_logger().info(
                f"targetPos grew {self.num_targets} -> {count}; new harvest cycle -- rearming mission state."
            )
            self.mission_done = False
            self.dump_requested = False
            self.dump_finished = False
        self.num_targets = count

    def on_mission_done(self, msg: Bool) -> None:
        if msg.data:
            self.mission_done = True

    def maybe_publish_mission_done(self) -> None:
        """Every rock either collected or skipped -> tell the driver to go home."""
        if self.mission_done or self.num_targets <= 0:
            return
        if len(self.completed_targets) < self.num_targets:
            return
        self.mission_done = True
        self.mission_done_pub.publish(Bool(data=True))
        self.get_logger().info(
            f"All {self.num_targets} targets resolved; published "
            f"{self.mission_done_topic}=true to return home and dump."
        )

    def on_at_home(self, msg: Bool) -> None:
        if not msg.data or self.dump_requested or not self.mission_done:
            return
        self.dump_requested = True
        # The cycle itself (gate, tilt, dwell, level, gate) runs in the simulation at
        # step rate, because its motors have to be slewed smoothly or the load is
        # flung rather than tipped. One request is enough; repeats are ignored there.
        self.trailer_cmd_pub.publish(Float64MultiArray(data=[1.0]))
        self.get_logger().info(f"At spawn; requested a dump cycle on {self.trailer_cmd_topic}.")

    def on_trailer_state(self, msg: Float64MultiArray) -> None:
        if not msg.data or self.dump_finished or not self.dump_requested:
            return
        # 6 == RobotRig::DumpState::DONE: bed level again and tailgate closed.
        if int(round(float(msg.data[0]))) == 6:
            self.dump_finished = True
            self.get_logger().info("Dump cycle finished: bed level, tailgate closed. Mission complete.")

    def publish_target_done(self, target_index: int) -> None:
        self.completed_targets.add(target_index)
        msg = Bool()
        msg.data = True
        self.target_done_pub.publish(msg)
        self.maybe_publish_mission_done()

    def parse_pickup_request(self, msg: Float64MultiArray) -> Optional[PickupRequest]:
        if len(msg.data) < 3:
            self.get_logger().warn(
                "Ignoring pickup_request; expected [target_index, rock_x_global, rock_y_global, optional rock_z_global]."
            )
            return None

        return PickupRequest(
            target_index=int(round(float(msg.data[0]))),
            rock_x=float(msg.data[1]),
            rock_y=float(msg.data[2]),
            rock_z=float(msg.data[3]) if len(msg.data) >= 4 else None,
        )

    def compute_grab_theta(self, request: PickupRequest) -> Optional[Tuple[float, float, float, float]]:
        if self.ik_solver is None:
            self.get_logger().warn("Cannot compute arm IK; RobotArmInverseKinematicsSolver is unavailable.")
            return None
        if self.arm_base_pose is None:
            self.get_logger().warn("Cannot compute arm IK yet; no arm_base_pose has been received.")
            return None

        grab_z_offset = float(self.get_parameter("grab_z_offset_m").value)
        grab_height = float(self.get_parameter("grab_height_m").value)
        grab_world_z = (
            request.rock_z + grab_z_offset
            if request.rock_z is not None
            else grab_height
        )
        local_target = self.world_to_arm_base_local(request.rock_x, request.rock_y, grab_world_z)
        try:
            theta = self.ik_solver.inverse_kinematics_solver(local_target, elbow_up=True)
        except ValueError as exc:
            # Log the arm-base frame and the ego pose alongside the local target. A
            # local target 10+ m out for a rock the rover is parked beside means the
            # frame is wrong, not the reach -- and only the frame's own numbers tell
            # the two apart. ego_dist is how far the base is from the vehicle origin;
            # they are metres apart on the same chassis, so a large value is the bug.
            base = self.arm_base_pose
            ego_text = ""
            if self.ego_state is not None and base is not None:
                ego_dist = math.hypot(base.x - self.ego_state.x, base.y - self.ego_state.y)
                ego_text = (
                    f", ego=({self.ego_state.x:.3f}, {self.ego_state.y:.3f}), base_to_ego={ego_dist:.3f} m"
                )
            self.get_logger().warn(
                f"Python IK failed for target {request.target_index}: {exc}; "
                f"local_target=({local_target[0]:.3f}, {local_target[1]:.3f}, {local_target[2]:.3f}), "
                f"rock=({request.rock_x:.3f}, {request.rock_y:.3f})"
                f"{f', base=({base.x:.3f}, {base.y:.3f}, {base.z:.3f})' if base else ''}"
                f"{ego_text}"
            )
            return None
        fk = self.ik_solver.forward_kinematics(theta)
        fk_err = math.sqrt(
            (float(fk[0]) - local_target[0]) ** 2
            + (float(fk[1]) - local_target[1]) ** 2
            + (float(fk[2]) - local_target[2]) ** 2
        )
        self.get_logger().info(
            f"Python IK target={request.target_index} "
            f"local_target=({local_target[0]:.3f}, {local_target[1]:.3f}, {local_target[2]:.3f}) "
            f"fk=({float(fk[0]):.3f}, {float(fk[1]):.3f}, {float(fk[2]):.3f}) "
            f"fk_err={fk_err:.4f}"
        )
        return tuple(float(value) for value in theta[:4])

    def compute_place_theta(self, target_index: int) -> Optional[Tuple[float, float, float, float]]:
        """Solve the drop pose with the same IK/frame as the grab, from the C++
        place target. Returns None (arm falls back to its own place IK) if the
        target or arm base pose is unavailable, or the point is unreachable."""
        if self.ik_solver is None or self.arm_base_pose is None or self.place_target is None:
            return None
        px, py, pz = self.place_target
        local_target = self.world_to_arm_base_local(px, py, pz)
        try:
            theta = self.ik_solver.inverse_kinematics_solver(local_target, elbow_up=True)
        except ValueError as exc:
            self.get_logger().warn(
                f"Python place IK failed for target {target_index}: {exc}; "
                f"place_local=({local_target[0]:.3f}, {local_target[1]:.3f}, {local_target[2]:.3f})"
            )
            return None
        return tuple(float(value) for value in theta[:4])

    def world_to_arm_base_local(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        base = self.arm_base_pose
        if base is None:
            raise RuntimeError("arm_base_pose is required")

        dx = x - base.x
        dy = y - base.y
        dz = z - base.z
        norm = math.sqrt(base.qw * base.qw + base.qx * base.qx + base.qy * base.qy + base.qz * base.qz)
        if norm <= 1e-12:
            return dx, dy, dz

        w = base.qw / norm
        qx = base.qx / norm
        qy = base.qy / norm
        qz = base.qz / norm

        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - w * qz)
        r02 = 2.0 * (qx * qz + w * qy)
        r10 = 2.0 * (qx * qy + w * qz)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - w * qx)
        r20 = 2.0 * (qx * qz - w * qy)
        r21 = 2.0 * (qy * qz + w * qx)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        return (
            r00 * dx + r10 * dy + r20 * dz,
            r01 * dx + r11 * dy + r21 * dz,
            r02 * dx + r12 * dy + r22 * dz,
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
