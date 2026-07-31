#include "RosTrailerBridge.h"

#include <algorithm>
#include <cmath>

#include "RobotRig.h"

namespace amd_uw {

namespace {

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

RosTrailerBridge::RosTrailerBridge(int robot_id, RobotRig& rig) : m_robot_id(robot_id), m_rig(rig) {
    EnsureRosInitialized();

    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_node = rclcpp::Node::make_shared("chrono_robot_" + std::to_string(m_robot_id) + "_trailer");
    m_state_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForRobot(m_robot_id, "trailer_state"), 10);
    m_command_sub = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(
        TopicForRobot(m_robot_id, "trailer_cmd"),
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnCommand(msg); });
    m_executor->add_node(m_node);

    RCLCPP_INFO(m_node->get_logger(),
                "Trailer bridge ready: subscribing %s, publishing %s",
                TopicForRobot(m_robot_id, "trailer_cmd").c_str(),
                TopicForRobot(m_robot_id, "trailer_state").c_str());
}

RosTrailerBridge::~RosTrailerBridge() {
    if (m_node && m_executor)
        m_executor->remove_node(m_node);
}

void RosTrailerBridge::Synchronize() {
    m_executor->spin_some();

    if (m_dump_requested) {
        m_dump_requested = false;
        if (m_rig.RequestTrailerDump())
            RCLCPP_INFO(m_node->get_logger(), "Dump cycle started for robot %d.", m_robot_id);
    }

    PublishState();
}

void RosTrailerBridge::OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_WARN(m_node->get_logger(), "Ignoring trailer_cmd; expected at least [command].");
        return;
    }
    if (!std::isfinite(msg->data[0])) {
        RCLCPP_WARN(m_node->get_logger(), "Ignoring trailer_cmd containing a non-finite value.");
        return;
    }

    // Only a dump request is defined. The cycle always returns the bed to level and
    // the gate to closed on its own, so there is no separate reset command.
    if (static_cast<int>(std::lround(msg->data[0])) == 1)
        m_dump_requested = true;
}

void RosTrailerBridge::PublishState() {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {static_cast<double>(static_cast<int>(m_rig.GetDumpState())),
                m_rig.GetBedAngle(),
                m_rig.GetTailgateAngle()};
    m_state_pub->publish(msg);
}

}  // namespace amd_uw
