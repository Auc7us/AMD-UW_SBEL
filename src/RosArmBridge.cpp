#include "RosArmBridge.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>

namespace amd_uw {

namespace {

// Aim the grab target at the rock's actual center (rock.GetPos()) plus a tiny
// vertical offset, so the target tracks each rock's real height instead of a
// fixed height above terrain. Must match manipulator_controller's grab_z_offset_m
// so the C++ target and the Python-solved theta agree.
constexpr double grab_z_offset = 0.05;
// Placement grid on the trailer bed, in the trailer's LOCAL frame (x = along the
// trailer, y = across it). Rocks tile across a bounded grid that fits inside the
// bed (~1.0 x 1.2 m) so they land on the bed at any heading, instead of a spread
// tied to the whole rock field that flung them meters off to the side.
constexpr double place_height = 0.5;  // local-z release height above the bed

// Duplicate-publisher detection: ignore the first seconds (DDS discovery, plus
// phantom publishers left by hard-killed nodes) and require the count to stay high
// across several spaced samples before believing it.
constexpr double dup_check_start_time = 3.0;
constexpr double dup_check_interval = 1.0;
constexpr int dup_check_confirmations = 3;

void EnsureRosInitialized() {
    if (rclcpp::ok())
        return;

    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
}

std::string TopicForRobot(int robot_id, const std::string& suffix) {
    return "/robot_" + std::to_string(robot_id) + "/" + suffix;
}

}  // namespace

RosArmBridge::RosArmBridge(int robot_id,
                           LrvArm& arm,
                           const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& rocks,
                           const std::vector<double>& rock_top_heights,
                           std::shared_ptr<chrono::vehicle::WheeledTrailer> trailer,
                           double height_probe_z)
    : m_robot_id(robot_id),
      m_arm(arm),
      m_rocks(rocks),
      m_rock_top_heights(rock_top_heights),
      m_trailer(std::move(trailer)),
      m_height_probe_z(height_probe_z) {
    EnsureRosInitialized();

    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_node = rclcpp::Node::make_shared("chrono_robot_" + std::to_string(m_robot_id) + "_arm");
    m_arm_base_pose_pub =
        m_node->create_publisher<std_msgs::msg::Float64MultiArray>(TopicForRobot(m_robot_id, "arm_base_pose"), 10);
    m_place_target_pub =
        m_node->create_publisher<std_msgs::msg::Float64MultiArray>(TopicForRobot(m_robot_id, "place_target"), 10);
    m_arm_status_pub =
        m_node->create_publisher<std_msgs::msg::Float64MultiArray>(TopicForRobot(m_robot_id, "arm_status"), 10);
    m_arm_cmd_sub = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(
        TopicForRobot(m_robot_id, "arm_cmd"),
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnArmCommand(msg); });
    m_executor->add_node(m_node);

    RCLCPP_INFO(m_node->get_logger(),
                "ROS arm bridge ready: subscribing %s, publishing %s",
                TopicForRobot(m_robot_id, "arm_cmd").c_str(),
                TopicForRobot(m_robot_id, "arm_status").c_str());
}

RosArmBridge::~RosArmBridge() {
    if (m_node && m_executor)
        m_executor->remove_node(m_node);
}

void RosArmBridge::Synchronize(double time, chrono::vehicle::ChTerrain& terrain) {
    m_last_time = time;
    m_executor->spin_some();

    // Duplicate manipulator nodes are worse here than on the drive topic. Each one
    // caches its own arm_base_pose and solves its own IK, so the same rock yields
    // several different answers arriving under different sequence numbers -- which
    // reads as an arm that cannot solve a rock it is parked next to, or that reaches
    // for empty ground. `ros2 launch` children outlive a kill of the launch process,
    // so this happens whenever a previous run was not torn down by process group.
    //
    // Must be a SUSTAINED count, not the first reading. A pkill -9 never lets a node
    // unregister from DDS discovery, so its publisher lingers in the graph for a lease
    // period after the process is gone -- and the first steps of the run are exactly
    // when that phantom is still visible. Warning on one sample cried wolf on a clean
    // graph right after a hard kill, which is precisely when this check is consulted.
    if (!m_multi_publisher_warned && time > dup_check_start_time &&
        time - m_dup_check_last_time >= dup_check_interval) {
        m_dup_check_last_time = time;
        if (m_arm_cmd_sub->get_publisher_count() > 1) {
            if (++m_dup_confirmations >= dup_check_confirmations) {
                m_multi_publisher_warned = true;
                RCLCPP_WARN(m_node->get_logger(),
                            "%zu publishers on %s for %.0f s -- expected 1. Leftover manipulator nodes each "
                            "solve IK in their own stale frame, and a second SIM on this ROS graph does the "
                            "same, so results are not trustworthy until it is gone. Check for both: "
                            "pgrep -af 'manipulator_controller|demo_SYN'",
                            m_arm_cmd_sub->get_publisher_count(), TopicForRobot(m_robot_id, "arm_cmd").c_str(),
                            dup_check_confirmations * dup_check_interval);
            }
        } else {
            m_dup_confirmations = 0;
        }
    }

    PublishArmBasePose(time);
    PublishPlaceTarget();

    std::optional<ArmCommand> command;
    {
        std::lock_guard<std::mutex> lock(m_command_mutex);
        command = m_pending_command;
        m_pending_command.reset();
    }

    if (command && !m_arm.IsBusy() && command->command_seq > m_last_started_seq) {
        m_last_started_seq = command->command_seq;
        if (command->target_index < 0 || command->target_index >= static_cast<int>(m_rocks.size())) {
            m_arm.StartPickPlace(command->command_seq,
                                 command->target_index,
                                 nullptr,
                                 chrono::VNULL,
                                 chrono::VNULL,
                                 time);
        } else {
            auto rock = m_rocks[command->target_index];
            const auto rock_pos = rock->GetPos();
            // Target the rock's real center height plus a tiny offset (matches the
            // Python controller), rather than a fixed height above the terrain.
            const chrono::ChVector3d grab_target(rock_pos.x(), rock_pos.y(), rock_pos.z() + grab_z_offset);
            const chrono::ChVector3d place_target = PlacePoint(m_place_count);
            RCLCPP_INFO(m_node->get_logger(),
                        "pickup start: target_index=%d mode=rock_center+offset "
                        "grab_z_offset=%.3f rock_z=%.3f grab_target=(%.3f, %.3f, %.3f) "
                        "theta=(%.3f, %.3f, %.3f, %.3f) "
                        "place_slot=%d place_target=(%.3f, %.3f, %.3f)",
                        command->target_index, grab_z_offset, rock_pos.z(),
                        grab_target.x(), grab_target.y(), grab_target.z(),
                        command->grab_theta[0], command->grab_theta[1], command->grab_theta[2],
                        command->grab_theta[3],
                        m_place_count, place_target.x(), place_target.y(), place_target.z());
            // Carry + stability check: is the previously placed rock still on the
            // trailer (small dist, elevated), and is the trailer upright (up.z ~ 1)?
            if (m_inflight_rock && m_trailer && m_trailer->GetChassis()) {
                auto chassis = m_trailer->GetChassis()->GetBody();
                const auto tp = chassis->GetPos();
                const auto rp = m_inflight_rock->GetPos();
                const double tz = terrain.GetHeight(chrono::ChVector3d(rp.x(), rp.y(), m_height_probe_z));
                const double up_z = chassis->GetRot().Rotate(chrono::ChVector3d(0, 0, 1)).z();
                RCLCPP_INFO(m_node->get_logger(),
                            "prev rock: dist_to_trailer_xy=%.3f height_above_terrain=%.3f | trailer up.z=%.3f",
                            std::hypot(rp.x() - tp.x(), rp.y() - tp.y()), rp.z() - tz, up_z);
            }
            m_inflight_rock = rock;

            m_arm.StartPickPlace(command->command_seq,
                                 command->target_index,
                                 rock,
                                 grab_target,
                                 place_target,
                                 time,
                                 &command->grab_theta,
                                 command->has_place_theta ? &command->place_theta : nullptr);
            m_place_count++;
        }
    }

    m_arm.Update(time);

    // On a successful place, keep the dropped rock awake so it doesn't settle-
    // freeze on the bed when the trailer next stops.
    const auto status = m_arm.GetStatus();
    if (status.state == 2 && status.success && status.command_seq == m_last_started_seq &&
        m_placed_seq != m_last_started_seq && m_inflight_rock) {
        m_inflight_rock->SetSleepingAllowed(false);
        m_inflight_rock->SetSleeping(false);
        m_placed_seq = m_last_started_seq;  // handled this placement once
    }

    // Throttled for the same reason as arm_base_pose, but it matters more here:
    // arm_status now carries the sim clock the manipulator controller uses as its
    // deadline. Published every 5e-4 s step, it buried the Python node in a backlog
    // it could never drain, so the controller's view of sim time crawled -- and its
    // sim-time timeout could never fire correctly because its clock barely moved.
    constexpr double status_period = 0.02;  // 50 Hz of sim time
    if (m_last_status_pub_time < 0.0 || time - m_last_status_pub_time >= status_period) {
        m_last_status_pub_time = time;
        PublishStatus();
    }
}

void RosArmBridge::OnArmCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 8) {
        RCLCPP_WARN(m_node->get_logger(),
                    "Ignoring arm_cmd; expected [command_seq, target_index, rock_x_global, rock_y_global, theta1..theta4].");
        return;
    }

    ArmCommand command;
    command.command_seq = msg->data[0];
    command.target_index = static_cast<int>(std::llround(msg->data[1]));
    command.rock_x = msg->data[2];
    command.rock_y = msg->data[3];
    command.grab_theta = {msg->data[4], msg->data[5], msg->data[6], msg->data[7]};
    // Optional Python-solved place pose (12-element command); otherwise the arm
    // falls back to its own place IK.
    if (msg->data.size() >= 12) {
        command.place_theta = {msg->data[8], msg->data[9], msg->data[10], msg->data[11]};
        command.has_place_theta = true;
    }

    std::lock_guard<std::mutex> lock(m_command_mutex);
    m_pending_command = command;
}

void RosArmBridge::PublishArmBasePose(double time) {
    // The arm base body is the IK frame origin. Its offset from the chassis is fixed
    // by a rigid mount, so any change means the base drifted -- and since every rock
    // is transformed into this frame, a drift of d moves every IK target by d and the
    // solver reports "unreachable" for a rock the arm is parked next to.
    const double offset = m_arm.BaseOffsetFromChassis();
    // Order matters: test for non-finite FIRST. `std::abs(NaN - ref) > 0.25` is
    // false, so the plain drift branch below reports the finite part of a blow-up
    // and then falls permanently silent at the exact moment the arm tears off --
    // which is worse than no detector at all, because the last thing it printed
    // was a merely-large number that looked survivable.
    if (!std::isfinite(offset)) {
        if (m_base_drift_reports < 10) {
            ++m_base_drift_reports;
            std::cout << "[RosArmBridge] robot " << m_robot_id << " arm base NON-FINITE at t=" << time
                      << " (reference offset was " << m_base_offset_reference
                      << " m); this rank's arm has diverged\n";
        }
    } else if (m_base_offset_reference < 0.0) {
        m_base_offset_reference = offset;
    } else if (std::abs(offset - m_base_offset_reference) > 0.25 && m_base_drift_reports < 10) {
        ++m_base_drift_reports;
        std::cout << "[RosArmBridge] robot " << m_robot_id << " arm base DRIFT at t=" << time << ": offset "
                  << m_base_offset_reference << " -> " << offset << " m from chassis\n";
    }

    // Throttle: the consumer is a 10 Hz Python node doing a blocking IK solve, and
    // this used to publish every 5e-4 s step. 50 Hz of sim time is far more than it
    // can use and 40x less traffic to wade through.
    constexpr double base_pose_period = 0.02;
    if (m_last_base_pose_pub_time >= 0.0 && time - m_last_base_pose_pub_time < base_pose_period)
        return;
    m_last_base_pose_pub_time = time;

    const auto pos = m_arm.GetIkFramePos();
    const auto rot = m_arm.GetIkFrameRot();
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        pos.x(),
        pos.y(),
        pos.z(),
        rot.e0(),
        rot.e1(),
        rot.e2(),
        rot.e3(),
    };
    m_arm_base_pose_pub->publish(msg);
}

void RosArmBridge::PublishPlaceTarget() {
    // World point of the NEXT drop slot, so the Python controller can solve the
    // place IK in the same frame it uses for the grab. m_place_count is the slot the
    // next accepted pickup will use, matching what StartPickPlace passes below.
    const auto p = PlacePoint(m_place_count);
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {p.x(), p.y(), p.z()};
    m_place_target_pub->publish(msg);
}

void RosArmBridge::PublishStatus() {
    const auto status = m_arm.GetStatus();
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        status.command_seq,
        static_cast<double>(status.state),
        static_cast<double>(status.target_index),
        status.success ? 1.0 : 0.0,
        static_cast<double>(status.error_code),
        // Simulation time, appended so a controller can supervise the arm on the clock
        // the arm actually advances on. A wall-clock deadline is meaningless here: the
        // sim runs ~19x slower than real time, and that ratio changes with rank count,
        // terrain cost and machine, so any fixed wall timeout is wrong somewhere.
        m_last_time,
    };
    m_arm_status_pub->publish(msg);
}

chrono::ChVector3d RosArmBridge::PlacePoint(int slot) const {
    if (!m_trailer || !m_trailer->GetChassis())
        return chrono::ChVector3d(0.0, 0.0, place_height);

    // EVERY rock goes to the middle of the bed, whatever slot it is.
    //
    // This used to spread loads over a 4x4 grid at 0.25 m pitch, which puts a corner slot
    // 0.53 m from bed centre. That spacing only makes sense if the arm can hit a slot, and
    // it cannot: the gripper is typically ~0.27 m from its place target at the moment it
    // opens (logged as |gripper-place| at PLACING->RELEASING), and the rock then falls
    // place_height before it lands. Measured |rock-place| over a run sits at 0.42-0.56 m.
    // Aim at a corner slot with that error budget and roughly half the misses go over the
    // side rather than into the bed.
    //
    // Aiming at the centre spends the same error budget against the bed's full half-width
    // instead of what is left after the slot offset, so the same miss lands inside. The
    // cost is that loads stack rather than spread, which the bed can absorb: a cycle
    // carries two rocks of ~0.23 m against a 0.5 m release height.
    //
    // `slot` is kept in the signature because the caller counts placements with it and the
    // status topic reports it; it simply no longer steers the aim point.
    (void)slot;
    // Expressed in the trailer's local frame and mapped to world, so the drop point stays
    // over the bed whatever way the trailer is pointing.
    const chrono::ChVector3d local(0.0, 0.0, place_height);
    return m_trailer->GetChassis()->GetBody()->TransformPointLocalToParent(local);
}

}  // namespace amd_uw
