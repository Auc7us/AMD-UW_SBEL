#include "BuilderArmRosBridge.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "LrvArm.h"

namespace amd_uw {

namespace {

// Vertical aim of the grab, relative to the rock's centre of mass. CALIBRATED, not
// derived -- the value is negative, which is not what reasoning about the arm predicted.
//
// The horizontal aim is already excellent: |gripper-rockREF|xy comes out at 0.006-0.017 m
// across all four builders. So when a grab fails with pad_force exactly 0 and the jaws
// shut to 0.0998 m on a 0.30 m rock, the whole miss is vertical, and the reported
// finger-to-rock distance at closure measures it directly. Two settings, 4 builders:
//
//     grab_z_offset = +0.05  ->  finger-rock 0.19-0.20 m at closure  (some grabs succeed)
//     grab_z_offset = +0.22  ->  finger-rock 0.28-0.34 m at closure  (none succeed)
//
// Raising the aim 0.17 m moved the gripper 0.15 m FURTHER from the rock, so the jaws were
// already above it, and the sag story that motivated +0.22 was simply wrong. The slope is
// near unity, so distance ~ |offset + 0.14| and the miss vanishes around -0.14. Backing
// off slightly from that leaves margin on both sides, since rocks in a heap differ in
// height by more than the residual error.
constexpr double grab_z_offset = -0.13;
// Release height above the terrain at the wall slot. The rock is let go from rest, so
// this is a short drop that lets it seat itself on the regolith rather than being
// pressed into it by the fingers.
constexpr double place_z_clearance = 0.20;

// Both at 50 Hz of sim time, matching the rover's bridge. The consumer is a 10 Hz Python
// node doing a blocking IK solve; publishing every 5e-4 s step is what buried it.
constexpr double build_publish_period = 0.02;

void EnsureRosInitialized() {
    if (rclcpp::ok())
        return;

    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
}

std::string TopicForBuilder(int builder_id, const std::string& suffix) {
    return "/builder_" + std::to_string(builder_id) + "/" + suffix;
}

}  // namespace

BuilderArmRosBridge::BuilderArmRosBridge(int builder_id,
                                         LrvArm& arm,
                                         std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> pile_rocks,
                                         std::vector<chrono::ChVector3d> wall_slots)
    : m_builder_id(builder_id),
      m_arm(arm),
      m_pile_rocks(std::move(pile_rocks)),
      m_wall_slots(std::move(wall_slots)) {
    EnsureRosInitialized();

    // Slot k's rock is m_pile_rocks[k]; a length mismatch would silently lay the wrong
    // rock on the wrong slot, so take the shorter of the two rather than trusting either.
    const size_t usable = std::min(m_pile_rocks.size(), m_wall_slots.size());
    m_pile_rocks.resize(usable);
    m_wall_slots.resize(usable);

    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_node = rclcpp::Node::make_shared("chrono_builder_" + std::to_string(m_builder_id) + "_arm");
    m_state_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "arm_state"), 10);
    m_base_pose_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "arm_base_pose"), 10);
    m_pick_target_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "pick_target"), 10);
    m_place_target_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "place_target"), 10);
    m_status_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "arm_status"), 10);
    m_command_sub = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "arm_cmd"),
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnCommand(msg); });
    m_executor->add_node(m_node);

    RCLCPP_INFO(m_node->get_logger(),
                "Builder arm bridge ready: %zu wall slots to lay from %zu heaped rocks; "
                "subscribing %s, publishing %s / %s / %s",
                m_wall_slots.size(), m_pile_rocks.size(),
                TopicForBuilder(m_builder_id, "arm_cmd").c_str(),
                TopicForBuilder(m_builder_id, "pick_target").c_str(),
                TopicForBuilder(m_builder_id, "place_target").c_str(),
                TopicForBuilder(m_builder_id, "arm_status").c_str());
}

BuilderArmRosBridge::~BuilderArmRosBridge() {
    if (m_node && m_executor)
        m_executor->remove_node(m_node);
}

int BuilderArmRosBridge::ReadySlot() const {
    if (!m_hull_parked)
        return -1;
    if (m_placed_count >= static_cast<int>(m_wall_slots.size()))
        return -1;
    if (!m_pile_rocks[m_placed_count])
        return -1;
    return m_placed_count;
}

void BuilderArmRosBridge::Synchronize(double time, bool apply_commands) {
    m_last_time = time;
    m_executor->spin_some();

    std::optional<DirectCommand> direct;
    std::optional<PickPlaceCommand> pick_place;
    {
        std::lock_guard<std::mutex> lock(m_command_mutex);
        direct = m_pending_direct;
        pick_place = m_pending_pick_place;
        m_pending_direct.reset();
        m_pending_pick_place.reset();
    }

    if (!apply_commands) {
        // Settle window: drop whatever arrived; the arm keeps the pose it already has.
        PublishBuildTopics(time);
        PublishStateThrottled(time);
        return;
    }

    if (direct) {
        m_arm.SetJointTargets(direct->theta);
        m_arm.SetFingerClosure(direct->finger_closure);
    }

    if (pick_place && !m_arm.IsBusy() && pick_place->command_seq > m_last_started_seq) {
        const int slot = ReadySlot();
        // Accept only a command for the slot actually being worked. A stale or duplicate
        // command would otherwise re-pick a rock already laid -- which, because that rock
        // was re-fixed on the wall, means welding the gripper to the wall.
        if (slot >= 0 && pick_place->target_index == slot) {
            m_last_started_seq = pick_place->command_seq;
            m_active_slot = slot;
            auto rock = m_pile_rocks[slot];
            const auto rock_pos = rock->GetPos();
            const chrono::ChVector3d grab_target(rock_pos.x(), rock_pos.y(), rock_pos.z() + grab_z_offset);
            const chrono::ChVector3d place_target = m_wall_slots[slot];
            RCLCPP_INFO(m_node->get_logger(),
                        "build step %d/%zu: rock=(%.3f, %.3f, %.3f) -> slot=(%.3f, %.3f, %.3f)",
                        slot + 1, m_wall_slots.size(), grab_target.x(), grab_target.y(), grab_target.z(),
                        place_target.x(), place_target.y(), place_target.z());
            m_arm.StartPickPlace(pick_place->command_seq, slot, rock, grab_target, place_target, time,
                                 &pick_place->grab_theta, &pick_place->place_theta);
        } else if (pick_place->target_index != slot) {
            RCLCPP_WARN(m_node->get_logger(),
                        "Ignoring pick/place for slot %d; the builder is %s.",
                        pick_place->target_index,
                        slot < 0 ? (m_hull_parked ? "out of rocks" : "still driving to station")
                                 : "working a different slot");
        }
    }

    // LrvArm::Update is issued once per step by BuilderRig, after this returns, so the
    // status read below is one 5e-4 s step old. That is deliberate -- two Update calls
    // in a step would advance the slew twice -- and a step of lag on booking a
    // completion changes nothing.

    // Book the completion once, on the transition. A laid rock is FIXED again: it is now
    // part of the wall, so it must not be nudged by the next rock landing beside it, and
    // it costs the solver nothing sitting there for the rest of the run. This is also
    // what guarantees the invariant the heap starts with -- at most one rock in this
    // rank's build is a dynamic body at any moment.
    const auto status = m_arm.GetStatus();
    if (m_active_slot >= 0 && status.command_seq == m_last_started_seq && m_settled_seq != m_last_started_seq &&
        (status.state == 2 || status.state == 3)) {
        m_settled_seq = m_last_started_seq;
        auto rock = m_pile_rocks[m_active_slot];
        if (status.state == 2 && status.success) {
            if (rock) {
                rock->SetFixed(true);
                rock->EnableCollision(true);
            }
            m_placed_count = m_active_slot + 1;
            RCLCPP_INFO(m_node->get_logger(), "laid rock %d of %zu; station advances to slot %d.",
                        m_placed_count, m_wall_slots.size(), m_placed_count);
        } else {
            // A slot that could not be served must not stall the whole course. Put the
            // rock back the way the heap holds them and move on; the gap in the wall is
            // honest about what happened.
            if (rock) {
                rock->SetFixed(true);
                rock->EnableCollision(true);
            }
            m_placed_count = m_active_slot + 1;
            RCLCPP_WARN(m_node->get_logger(),
                        "slot %d failed (error_code=%d); skipping it and moving to slot %d.",
                        m_active_slot, status.error_code, m_placed_count);
        }
        // This rock's state is ours now. Tell the arm to let go of its reference, or the
        // next StartPickPlace's RemoveRockLock() would unfix the stone we just laid.
        m_arm.ForgetTargetRock();
        m_active_slot = -1;
    }

    if (!m_reported_complete && BuildComplete()) {
        m_reported_complete = true;
        RCLCPP_INFO(m_node->get_logger(), "course complete: %zu rocks laid on the work circle.",
                    m_wall_slots.size());
    }

    PublishBuildTopics(time);
    PublishStateThrottled(time);
}

void BuilderArmRosBridge::OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!std::all_of(msg->data.begin(), msg->data.end(), [](double value) { return std::isfinite(value); })) {
        RCLCPP_WARN(m_node->get_logger(), "Ignoring arm_cmd containing a non-finite value.");
        return;
    }

    if (msg->data.size() == 5) {
        DirectCommand command;
        command.theta = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};
        // Matches LrvArm's physical finger travel used by the reference model.
        command.finger_closure = std::clamp(msg->data[4], 0.0, 0.05);
        std::lock_guard<std::mutex> lock(m_command_mutex);
        m_pending_direct = command;
        return;
    }

    if (msg->data.size() >= 12) {
        PickPlaceCommand command;
        command.command_seq = msg->data[0];
        command.target_index = static_cast<int>(std::llround(msg->data[1]));
        // data[2..3] are the rock's world x/y, carried for symmetry with the rover's
        // command and for logging; the authoritative position is read off the body here.
        command.grab_theta = {msg->data[4], msg->data[5], msg->data[6], msg->data[7]};
        command.place_theta = {msg->data[8], msg->data[9], msg->data[10], msg->data[11]};
        std::lock_guard<std::mutex> lock(m_command_mutex);
        m_pending_pick_place = command;
        return;
    }

    RCLCPP_WARN(m_node->get_logger(),
                "Ignoring arm_cmd of %zu elements; expected 5 ([theta1..4, finger_closure_m]) or "
                "12 ([seq, target_index, rock_x, rock_y, grab_theta1..4, place_theta1..4]).",
                msg->data.size());
}

void BuilderArmRosBridge::PublishBuildTopics(double time) {
    if (m_last_build_pub_time >= 0.0 && time - m_last_build_pub_time < build_publish_period)
        return;
    m_last_build_pub_time = time;

    const auto pos = m_arm.GetIkFramePos();
    const auto rot = m_arm.GetIkFrameRot();
    std_msgs::msg::Float64MultiArray base;
    base.data = {pos.x(), pos.y(), pos.z(), rot.e0(), rot.e1(), rot.e2(), rot.e3()};
    m_base_pose_pub->publish(base);

    // ready=0 means "do not solve": the hull is still driving to station, the course is
    // finished, or the heap is empty. The x/y/z are still sent so a stalled builder can
    // be diagnosed from the topic instead of from the log.
    const int slot = ReadySlot();
    const int slot_for_geometry = std::min(std::max(m_placed_count, 0),
                                           static_cast<int>(m_wall_slots.size()) - 1);
    std_msgs::msg::Float64MultiArray pick;
    if (slot_for_geometry >= 0 && m_pile_rocks[slot_for_geometry]) {
        const auto rock_pos = m_pile_rocks[slot_for_geometry]->GetPos();
        pick.data = {slot >= 0 ? 1.0 : 0.0, static_cast<double>(slot_for_geometry), rock_pos.x(), rock_pos.y(),
                     rock_pos.z() + grab_z_offset};
    } else {
        pick.data = {0.0, -1.0, 0.0, 0.0, 0.0};
    }
    m_pick_target_pub->publish(pick);

    std_msgs::msg::Float64MultiArray place;
    if (slot_for_geometry >= 0) {
        const auto& p = m_wall_slots[slot_for_geometry];
        place.data = {p.x(), p.y(), p.z()};
    } else {
        place.data = {0.0, 0.0, 0.0};
    }
    m_place_target_pub->publish(place);

    const auto status = m_arm.GetStatus();
    std_msgs::msg::Float64MultiArray status_msg;
    status_msg.data = {
        status.command_seq,
        static_cast<double>(status.state),
        static_cast<double>(status.target_index),
        status.success ? 1.0 : 0.0,
        static_cast<double>(status.error_code),
        // Simulation time. The arm advances on the sim clock, so that is the only clock
        // a supervising controller's deadline can sensibly use -- wall time is ~20x
        // faster here and the ratio moves with rank count and machine.
        m_last_time,
    };
    m_status_pub->publish(status_msg);
}

// See the note in BuilderVehicleRosBridge::Synchronize: publishing arm_state at the
// 2 kHz step rate was part of what saturated Fast-DDS.
void BuilderArmRosBridge::PublishStateThrottled(double time) {
    constexpr double publish_period = 1.0 / 200.0;
    if (m_last_publish_time >= 0.0 && time - m_last_publish_time < publish_period)
        return;
    m_last_publish_time = time;
    PublishState();
}

void BuilderArmRosBridge::PublishState() {
    const auto state = m_arm.GetActuatorSnapshot();
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        state.joint_angles[0],
        state.joint_angles[1],
        state.joint_angles[2],
        state.joint_angles[3],
        state.finger_positions[0],
        state.finger_positions[1],
        state.end_effector_position.x(),
        state.end_effector_position.y(),
        state.end_effector_position.z(),
    };
    m_state_pub->publish(msg);
}

}  // namespace amd_uw
