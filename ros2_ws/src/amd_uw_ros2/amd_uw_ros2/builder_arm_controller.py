"""Builder arm: lay a course of rocks on the work circle, one slot at a time.

The builder runs this on its OWN schedule. Nothing here knows or cares what its
collector is doing -- no harvest cycle, no dump, no mission_done. The loop is:

    sim says "parked, slot k, rock is there"  ->  solve grab + place IK
    -> send one pick-and-place  ->  wait for arm_status  ->  sim advances the station
    -> the hull creeps ~1 m along its lane  ->  repeat at slot k+1

so the wall grows stone by stone, counter-clockwise, and the machine moves a little
between each one. Every builder is doing the same thing on its own arc at the same
time.

WHY THE TARGETS COME FROM THE SIM AND NOT FROM GEOMETRY HERE. An earlier version
computed both targets from site constants, which meant this node believed the builder
was exactly on its nominal station. It is not: it is a tracked vehicle station-keeping
on sloped regolith, and it stops wherever it stops. `/builder_N/pick_target` is read off
the rock body itself and `/builder_N/arm_base_pose` off the arm's base body, so the IK
is solved against where things ACTUALLY are. This is privileged information on purpose
-- the task being demonstrated is construction, not perception.

The solver is RobotArmInverseKinematicsSolver(scale=2.0): the builder's arm is the LRV
arm at 2x geometric scale, a1..a4 = 0.650, 2.540, 2.286, 0.715. Only the solver scales;
the finger geometry and the mass/inertia values stay 1x (see BuilderRig), and so do the
finger-related limits in LrvArm.
"""

import math
import time as wall_time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from amd_uw_ros2.inverse_kinematics import RobotArmInverseKinematicsSolver


ARM_STATE_IDLE = 0
ARM_STATE_BUSY = 1
ARM_STATE_DONE = 2
ARM_STATE_FAILED = 3


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
class PickTarget:
    ready: bool
    index: int
    x: float
    y: float
    z: float


@dataclass
class ActiveCommand:
    command_seq: int
    slot: int
    last_publish_wall_s: float
    status_seen: bool = False


class BuilderArmController(Node):
    """Drive one tracked builder's arm through its wall-building cycle."""

    def __init__(self) -> None:
        super().__init__("builder_arm_controller")

        self.declare_parameter("builder_id", 1)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("cycle_enabled", True)

        # Must match the physical LrvArm geometry scale used by BuilderRig.
        self.declare_parameter("arm_scale", 2.0)

        # Reach guard, the LRV's proven 1.39-2.22 m band scaled by arm_scale with margin.
        # This is a pre-check only; LrvArm enforces its own scaled envelope before it
        # commands a motor, and that one is authoritative.
        self.declare_parameter("min_reach_m", 2.0)
        self.declare_parameter("max_reach_m", 5.2)
        self.declare_parameter("ik_tolerance_m", 0.02)

        # The real deadline, on the SIM clock reported by arm_status: the sim runs ~20x
        # slower than wall and that ratio moves with rank count and machine, so no fixed
        # wall number is ever right. A full pick-and-place is ~15-20 s of sim time.
        self.declare_parameter("command_timeout_sim_s", 90.0)
        # Wall-clock BACKSTOP for "the sim is not progressing at all". A liveness check,
        # not a duration budget, so it is deliberately generous.
        self.declare_parameter("command_timeout_s", 1800.0)
        self.declare_parameter("cmd_republish_rate_hz", 1.0)

        self.builder_id = int(self.get_parameter("builder_id").value)
        self.command_topic = f"/builder_{self.builder_id}/arm_cmd"
        self.base_pose_topic = f"/builder_{self.builder_id}/arm_base_pose"
        self.pick_topic = f"/builder_{self.builder_id}/pick_target"
        self.place_topic = f"/builder_{self.builder_id}/place_target"
        self.status_topic = f"/builder_{self.builder_id}/arm_status"

        # The sim ignores commands with old sequence numbers. Seed from a monotonic clock
        # so restarting this node does not replay seq=1 and get silently dropped.
        self.command_seq = int(wall_time.monotonic_ns() // 1_000_000)
        self.active: Optional[ActiveCommand] = None
        self.active_start_wall_s: Optional[float] = None
        self.active_start_sim_s: Optional[float] = None
        self.sim_time: Optional[float] = None

        self.arm_base_pose: Optional[ArmBasePose] = None
        self.pick_target: Optional[PickTarget] = None
        self.place_target: Optional[Tuple[float, float, float]] = None
        self.rocks_laid = 0
        self.reported_waiting = False
        self.reported_unsolvable_slot = -1
        # The poses of the in-flight command. Re-solved on every republish rather than
        # resent verbatim -- see supervise_active -- and kept only as the fallback for a
        # republish whose fresh solve failed.
        self.pending_theta = None

        self.solver = RobotArmInverseKinematicsSolver(
            scale=float(self.get_parameter("arm_scale").value)
        )

        self.command_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.create_subscription(Float64MultiArray, self.base_pose_topic, self.on_base_pose, 10)
        self.create_subscription(Float64MultiArray, self.pick_topic, self.on_pick_target, 10)
        self.create_subscription(Float64MultiArray, self.place_topic, self.on_place_target, 10)
        self.create_subscription(Float64MultiArray, self.status_topic, self.on_status, 10)

        rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            f"builder_{self.builder_id} arm: {self.pick_topic} + {self.place_topic} -> "
            f"{self.command_topic}, arm_scale={self.get_parameter('arm_scale').value}"
        )

    # ------------------------------------------------------------------ inputs

    def on_base_pose(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 7:
            self.get_logger().warn("Ignoring arm_base_pose; expected [x, y, z, qw, qx, qy, qz].")
            return
        self.arm_base_pose = ArmBasePose(*(float(v) for v in msg.data[:7]))

    def on_pick_target(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 5:
            self.get_logger().warn("Ignoring pick_target; expected [ready, index, x, y, z].")
            return
        self.pick_target = PickTarget(
            ready=bool(round(float(msg.data[0]))),
            index=int(round(float(msg.data[1]))),
            x=float(msg.data[2]),
            y=float(msg.data[3]),
            z=float(msg.data[4]),
        )

    def on_place_target(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warn("Ignoring place_target; expected [x, y, z].")
            return
        self.place_target = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

    def on_status(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 5:
            self.get_logger().warn(
                "Ignoring arm_status; expected [seq, state, index, success, error_code, sim_time]."
            )
            return

        command_seq = int(round(float(msg.data[0])))
        state = int(round(float(msg.data[1])))
        slot = int(round(float(msg.data[2])))
        success = bool(round(float(msg.data[3])))
        error_code = int(round(float(msg.data[4])))
        if len(msg.data) >= 6:
            self.sim_time = float(msg.data[5])

        if self.active is None or command_seq != self.active.command_seq:
            return

        self.active.status_seen = True
        # Anchor the deadline to the first sim time seen for THIS command, not to when it
        # was sent: the sim may not have picked it up yet.
        if self.active_start_sim_s is None and self.sim_time is not None:
            self.active_start_sim_s = self.sim_time

        if state == ARM_STATE_DONE and success:
            self.rocks_laid += 1
            self.get_logger().info(
                f"laid rock {self.rocks_laid} on slot {slot}; moving up the lane."
            )
            self.clear_active()
        elif state == ARM_STATE_FAILED:
            self.get_logger().warn(
                f"slot {slot} failed with error_code={error_code}; the sim skips it and "
                f"advances the station."
            )
            self.clear_active()

    # ------------------------------------------------------------------- cycle

    def on_timer(self) -> None:
        if self.command_pub.get_subscription_count() == 0:
            if not self.reported_waiting:
                self.get_logger().warn(f"Waiting for the simulation subscriber on {self.command_topic}.")
                self.reported_waiting = True
            return
        self.reported_waiting = False

        if self.active is not None:
            self.supervise_active()
            return

        if not bool(self.get_parameter("cycle_enabled").value):
            return
        if self.arm_base_pose is None or self.pick_target is None or self.place_target is None:
            return
        # ready=0 means the sim is telling us not to solve: the hull is still creeping to
        # its next station, or the course is finished. Solving anyway would produce a pose
        # in a frame that is about to move.
        if not self.pick_target.ready:
            return

        slot = self.pick_target.index
        grab = self.solve("grab", (self.pick_target.x, self.pick_target.y, self.pick_target.z))
        place = self.solve("place", self.place_target)
        if grab is None or place is None:
            # Do NOT publish. The sim only advances the station on a completed or failed
            # ATTEMPT, so staying silent leaves the builder parked here, and every log
            # line names the slot and the local target that could not be solved. Silently
            # skipping would leave an unexplained gap in the wall.
            if slot != self.reported_unsolvable_slot:
                self.reported_unsolvable_slot = slot
                self.get_logger().error(
                    f"slot {slot}: no usable pose; holding station rather than commanding it."
                )
            return

        self.command_seq += 1
        now = self.now_wall_s()
        self.active = ActiveCommand(self.command_seq, slot, now)
        self.active_start_wall_s = now
        self.active_start_sim_s = self.sim_time
        self.pending_theta = (grab, place)
        self.publish_active(grab, place)
        self.get_logger().info(
            f"slot {slot}: rock=({self.pick_target.x:.2f}, {self.pick_target.y:.2f}, "
            f"{self.pick_target.z:.2f}) -> wall=({self.place_target[0]:.2f}, "
            f"{self.place_target[1]:.2f}, {self.place_target[2]:.2f}); "
            f"grab={[round(t, 3) for t in grab]} place={[round(t, 3) for t in place]}"
        )

    def supervise_active(self) -> None:
        if self.active is None:
            return

        # Republish until the sim acknowledges. A command that arrives while the arm is
        # still stowing from the previous slot is dropped, and nothing else would resend.
        #
        # RE-SOLVE, do not resend. Joint angles are only meaningful in the arm base frame
        # they were solved in, and that frame moves: the hull creeps under full brake, and
        # BuilderRig's anchor deliberately walks it back onto the lane at 0.15 m/s. A
        # command can sit unacknowledged for seconds while that happens -- the sim only
        # accepts one from a parked hull -- so resending the original angles aims the
        # gripper at where the rock USED to be relative to the base.
        #
        # Measured, tagged, with the miss split by axis:
        #   [LrvArm builder_2_] GRAB FAILED(3) miss_xy=0.726 miss_z=-0.082
        #       rock=(-3.04275, 35.9807, 1.88634)  aim=(-3.04275, 35.9807, 1.75634)
        # The aim point is exactly the rock's x/y and z-0.13, so the target was right and
        # the height was right; the gripper simply went 0.73 m sideways of it, which is a
        # stale frame and nothing else. Re-solving costs one BFGS minimisation per second.
        rate_hz = max(0.0, float(self.get_parameter("cmd_republish_rate_hz").value))
        if (
            rate_hz > 0.0
            and not self.active.status_seen
            and self.now_wall_s() - self.active.last_publish_wall_s >= 1.0 / rate_hz
        ):
            # Re-solve ONLY against a target the sim is still offering. ready=0 means the
            # bridge has no rock selected, and its pick_target then carries (0, 0, 0) --
            # the site centre, which from an arm base on the 33 m lane is a target 31 m
            # away that the reach guard rejects once per republish. Honouring the flag here
            # as the first-solve path already does is the whole fix; falling back to the
            # last good pose keeps the republish doing its job, which is to cover a command
            # the sim dropped because the arm was still stowing.
            fresh = None
            if (
                self.arm_base_pose is not None
                and self.pick_target is not None
                and self.pick_target.ready
                and self.place_target is not None
            ):
                grab = self.solve("grab", (self.pick_target.x, self.pick_target.y, self.pick_target.z))
                place = self.solve("place", self.place_target)
                if grab is not None and place is not None:
                    fresh = (grab, place)
            if fresh is not None:
                self.pending_theta = fresh
            if self.pending_theta is not None:
                self.publish_active(*self.pending_theta)

        sim_timeout = max(0.0, float(self.get_parameter("command_timeout_sim_s").value))
        if sim_timeout > 0.0 and self.sim_time is not None and self.active_start_sim_s is not None:
            elapsed_sim = self.sim_time - self.active_start_sim_s
            if elapsed_sim > sim_timeout:
                self.get_logger().warn(
                    f"slot {self.active.slot} timed out after {elapsed_sim:.1f} s of SIM time "
                    f"(limit {sim_timeout:.1f}); giving up on this command."
                )
                self.clear_active()
                return

        wall_timeout = max(0.0, float(self.get_parameter("command_timeout_s").value))
        if wall_timeout <= 0.0 or self.active_start_wall_s is None:
            return
        if self.now_wall_s() - self.active_start_wall_s > wall_timeout:
            stalled = (
                "no arm_status ever arrived"
                if not self.active.status_seen
                else f"sim clock stuck at {self.sim_time}"
            )
            self.get_logger().warn(
                f"slot {self.active.slot} hit the {wall_timeout:.0f} s WALL backstop ({stalled}). "
                f"This means the sim is not progressing, not that the arm is slow."
            )
            self.clear_active()

    def clear_active(self) -> None:
        self.active = None
        self.active_start_wall_s = None
        self.active_start_sim_s = None
        self.pending_theta = None

    # --------------------------------------------------------------------- IK

    def solve(self, label: str, world: Tuple[float, float, float]):
        """IK for a world point, in the arm base frame, validated before it is trusted.

        The solver is an unconstrained BFGS minimisation of the FK residual: no joint
        limits, no reach limits. It will happily return a pose for a point underneath the
        arm's own base by folding the arm back through the hull, and report fk_err
        0.0000 -- which only ever meant "self-consistent", never "legal". These actuators
        are constraint motors with no torque ceiling, so an illegal pose drives the
        collision-enabled fingers into the hull and that reaction goes into the weld
        holding the arm base on.
        """
        local = self.world_to_arm_base_local(*world)
        reach = math.hypot(local[0], local[1])
        min_reach = float(self.get_parameter("min_reach_m").value)
        max_reach = float(self.get_parameter("max_reach_m").value)
        if not (min_reach <= reach <= max_reach):
            self.get_logger().warn(
                f"{label}: local=({local[0]:.3f}, {local[1]:.3f}, {local[2]:.3f}) is "
                f"{reach:.2f} m from the arm base, outside the {min_reach:.1f}-{max_reach:.1f} m "
                f"band. Refusing to command it."
            )
            return None
        try:
            theta = self.solver.inverse_kinematics_solver(local, elbow_up=True)
        except ValueError as exc:
            base = self.arm_base_pose
            self.get_logger().warn(
                f"{label}: IK failed: {exc}; local=({local[0]:.3f}, {local[1]:.3f}, "
                f"{local[2]:.3f}), world=({world[0]:.3f}, {world[1]:.3f}, {world[2]:.3f})"
                + (f", base=({base.x:.3f}, {base.y:.3f}, {base.z:.3f})" if base else "")
            )
            return None
        fk = self.solver.forward_kinematics(theta)
        error = math.sqrt(sum((float(fk[i]) - local[i]) ** 2 for i in range(3)))
        if error > float(self.get_parameter("ik_tolerance_m").value):
            self.get_logger().warn(f"{label}: IK residual {error:.4f} m; refusing to command it.")
            return None
        return tuple(float(v) for v in theta[:4])

    def world_to_arm_base_local(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        base = self.arm_base_pose
        if base is None:
            raise RuntimeError("arm_base_pose is required")

        dx, dy, dz = x - base.x, y - base.y, z - base.z
        norm = math.sqrt(base.qw**2 + base.qx**2 + base.qy**2 + base.qz**2)
        if norm <= 1e-12:
            return dx, dy, dz
        w, qx, qy, qz = base.qw / norm, base.qx / norm, base.qy / norm, base.qz / norm

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

    # ---------------------------------------------------------------- outputs

    def publish_active(self, grab, place) -> None:
        if self.active is None:
            return
        msg = Float64MultiArray()
        msg.data = [
            float(self.active.command_seq),
            float(self.active.slot),
            float(self.pick_target.x if self.pick_target else 0.0),
            float(self.pick_target.y if self.pick_target else 0.0),
            *(float(t) for t in grab),
            *(float(t) for t in place),
        ]
        self.command_pub.publish(msg)
        self.active.last_publish_wall_s = self.now_wall_s()

    def now_wall_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


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
