#include "RosArmBridge.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace amd_uw {

namespace {

constexpr double grab_height = 0.22;
constexpr double place_height = 0.5;
constexpr double place_spread_y = 0.4;

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
                           std::shared_ptr<chrono::vehicle::WheeledTrailer> trailer,
                           double height_probe_z)
    : m_robot_id(robot_id),
      m_arm(arm),
      m_rocks(rocks),
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
            const auto rock_pos = rock->GetPos();
            const double terrain_z = terrain.GetHeight(chrono::ChVector3d(rock_pos.x(), rock_pos.y(), m_height_probe_z));
            const chrono::ChVector3d grab_target(rock_pos.x(), rock_pos.y(), terrain_z + grab_height);
            m_arm.StartPickPlace(command->command_seq,
                                 command->target_index,
                                 rock,
                                 grab_target,
                                 PlacePoint(command->target_index),
                                 time);
        }
    }

    m_arm.Update(time);
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

chrono::ChVector3d RosArmBridge::PlacePoint(int target_index) const {
    if (!m_trailer || !m_trailer->GetChassis())
        return chrono::ChVector3d(0.0, 0.0, place_height);

    const int n = std::max(1, static_cast<int>(m_rocks.size()));
    const double lateral = (target_index - 0.5 * (n - 1)) * place_spread_y;
    return m_trailer->GetChassis()->GetBody()->GetPos() + chrono::ChVector3d(0.0, lateral, place_height);
}

}  // namespace amd_uw
