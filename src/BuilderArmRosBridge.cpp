#include "BuilderArmRosBridge.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "LrvArm.h"

namespace amd_uw {

namespace {

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

BuilderArmRosBridge::BuilderArmRosBridge(int builder_id, LrvArm& arm)
    : m_builder_id(builder_id), m_arm(arm) {
    EnsureRosInitialized();

    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_node = rclcpp::Node::make_shared("chrono_builder_" + std::to_string(m_builder_id) + "_arm");
    m_state_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "arm_state"), 10);
    m_command_sub = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "arm_cmd"),
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnCommand(msg); });
    m_executor->add_node(m_node);

    RCLCPP_INFO(m_node->get_logger(),
                "Builder arm bridge ready: subscribing %s, publishing %s",
                TopicForBuilder(m_builder_id, "arm_cmd").c_str(),
                TopicForBuilder(m_builder_id, "arm_state").c_str());
}

BuilderArmRosBridge::~BuilderArmRosBridge() {
    if (m_node && m_executor)
        m_executor->remove_node(m_node);
}

void BuilderArmRosBridge::Synchronize() {
    m_executor->spin_some();

    if (m_pending_command) {
        m_arm.SetJointTargets(m_pending_command->theta);
        m_arm.SetFingerClosure(m_pending_command->finger_closure);
        m_pending_command.reset();
    }

    PublishState();
}

void BuilderArmRosBridge::OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != 5) {
        RCLCPP_WARN(m_node->get_logger(),
                    "Ignoring arm_cmd; expected exactly "
                    "[theta1, theta2, theta3, theta4, finger_closure_m].");
        return;
    }

    if (!std::all_of(msg->data.begin(), msg->data.end(), [](double value) {
            return std::isfinite(value);
        })) {
        RCLCPP_WARN(m_node->get_logger(), "Ignoring arm_cmd containing a non-finite value.");
        return;
    }

    Command command;
    command.theta = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};
    // Matches LrvArm's physical finger travel used by the reference model.
    command.finger_closure = std::clamp(msg->data[4], 0.0, 0.05);
    m_pending_command = command;
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
