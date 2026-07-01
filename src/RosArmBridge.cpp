#include "RosArmBridge.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace amd_uw {

namespace {

// Fallback gripper-center height used only if a rock's measured height is
// missing. With the per-rock height available (the common path) the gripper
// aims at the rock's top plus this margin, reproducing the reference demo's
// gripper-center-at-the-rock-top grasp for rocks of any size/mesh.
constexpr double grab_height_fallback = 0.22;
constexpr double grab_top_margin = 0.0;
// Placement grid on the trailer bed, in the trailer's LOCAL frame (x = along the
// trailer, y = across it). Rocks tile across a bounded grid that fits inside the
// bed (~1.0 x 1.2 m) so they land on the bed at any heading, instead of a spread
// tied to the whole rock field that flung them meters off to the side.
// Welding placed rocks to the trailer keeps them from being left behind, but a
// rigid rock locked to the suspended trailer chassis destabilizes it (rolls a
// wheel up and flips under drive). Disabled until carry is done a stable way
// (e.g. near-massless rock, or a proper hinged bed like the Python demo).
constexpr bool enable_bed_weld = false;
constexpr double place_height = 0.5;  // local-z release height above the bed
constexpr int place_cols = 4;         // lateral slots (across the bed, y)
constexpr int place_rows = 4;         // longitudinal slots (along the bed, x)
constexpr double place_step_y = 0.25;
constexpr double place_step_x = 0.25;

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

void RosArmBridge::Synchronize(double time, chrono::vehicle::RigidTerrain& terrain) {
    m_executor->spin_some();

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
            // Aim at the rock's geometric center (REF frame), not GetPos(), which
            // for a ChBodyAuxRef is the center of mass -- offset from the visible
            // center by the mesh's COM offset (and the rock's random yaw).
            const auto rock_ref = rock->GetFrameRefToAbs().GetPos();
            const double terrain_z =
                terrain.GetHeight(chrono::ChVector3d(rock_ref.x(), rock_ref.y(), m_height_probe_z));
            // Height above the ground: reach for this rock's actual top (per-mesh),
            // falling back to the legacy fixed height only if it wasn't measured.
            const double grab_z =
                command->target_index < static_cast<int>(m_rock_top_heights.size())
                    ? m_rock_top_heights[command->target_index] + grab_top_margin
                    : grab_height_fallback;
            const chrono::ChVector3d grab_target(rock_ref.x(), rock_ref.y(), terrain_z + grab_z);
            const chrono::ChVector3d place_target = PlacePoint(m_place_count);
            RCLCPP_INFO(m_node->get_logger(),
                        "pickup start: target_index=%d rock_top_height=%.3f grab_z(rel ground)=%.3f "
                        "grab_target=(%.3f, %.3f, %.3f) place_slot=%d place_target=(%.3f, %.3f, %.3f)",
                        command->target_index, grab_z - grab_top_margin, grab_z, grab_target.x(),
                        grab_target.y(), grab_target.z(), m_place_count, place_target.x(), place_target.y(),
                        place_target.z());
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
                                 time);
            m_place_count++;
        }
    }

    m_arm.Update(time);

    // On a successful place, weld the rock to the trailer so it rides along
    // instead of being left behind (the bed is teleported and can't carry it by
    // friction). Keep it awake so it doesn't freeze mid-air after settling.
    const auto status = m_arm.GetStatus();
    if (status.state == 2 && status.success && status.command_seq == m_last_started_seq &&
        m_welded_seq != m_last_started_seq && m_inflight_rock && m_trailer && m_trailer->GetChassis()) {
        // Keep the placed rock awake so it rides the dynamic bed via friction
        // instead of napping and freezing in place when the trailer next stops.
        m_inflight_rock->SetSleepingAllowed(false);
        m_inflight_rock->SetSleeping(false);

        if (enable_bed_weld) {
            auto chassis = m_trailer->GetChassis()->GetBody();
            if (auto* sys = chassis->GetSystem()) {
                auto weld = chrono_types::make_shared<chrono::ChLinkLockLock>();
                weld->Initialize(chassis, m_inflight_rock,
                                 chrono::ChFramed(m_inflight_rock->GetPos(), chrono::QUNIT));
                sys->AddLink(weld);
                m_inflight_rock->EnableCollision(false);
                m_bed_welds.push_back(weld);
                m_welded_rocks.push_back(m_inflight_rock);
                RCLCPP_INFO(m_node->get_logger(), "welded placed rock to trailer (total on bed: %zu)",
                            m_bed_welds.size());
            }
        }
        m_welded_seq = m_last_started_seq;  // handled this placement once
    }

    PublishStatus();
}

void RosArmBridge::OnArmCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) {
        RCLCPP_WARN(m_node->get_logger(),
                    "Ignoring arm_cmd; expected [command_seq, target_index, rock_x_global, rock_y_global].");
        return;
    }

    ArmCommand command;
    command.command_seq = msg->data[0];
    command.target_index = static_cast<int>(std::llround(msg->data[1]));
    command.rock_x = msg->data[2];
    command.rock_y = msg->data[3];

    std::lock_guard<std::mutex> lock(m_command_mutex);
    m_pending_command = command;
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
    };
    m_arm_status_pub->publish(msg);
}

chrono::ChVector3d RosArmBridge::PlacePoint(int slot) const {
    if (!m_trailer || !m_trailer->GetChassis())
        return chrono::ChVector3d(0.0, 0.0, place_height);

    // Grid slot on the bed (wraps if more rocks than slots), centered on the bed.
    const int col = slot % place_cols;
    const int row = (slot / place_cols) % place_rows;
    const double y_local = (col - 0.5 * (place_cols - 1)) * place_step_y;
    const double x_local = (row - 0.5 * (place_rows - 1)) * place_step_x;
    // Express in the trailer's local frame and map to world, so the drop point
    // stays over the bed whatever way the trailer is pointing.
    const chrono::ChVector3d local(x_local, y_local, place_height);
    return m_trailer->GetChassis()->GetBody()->TransformPointLocalToParent(local);
}

}  // namespace amd_uw
