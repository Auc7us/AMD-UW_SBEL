#include "RosControllerDriver.h"

#include <algorithm>
#include <cmath>

#include "chrono/core/ChQuaternion.h"

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

RosControllerDriver::RosControllerDriver(chrono::vehicle::ChVehicle& vehicle,
                                         int robot_id,
                                         double state_publish_rate_hz)
    : chrono::vehicle::ChDriver(vehicle),
      m_robot_id(robot_id),
      m_state_publish_period(state_publish_rate_hz > 0 ? 1.0 / state_publish_rate_hz : 0.02),
      m_last_state_publish_time(-1.0),
      m_command_received(false) {
    EnsureRosInitialized();

    m_steering = 0.0;
    m_throttle = 0.0;
    m_braking = 1.0;
    m_clutch = 0.0;

    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_node = rclcpp::Node::make_shared("chrono_robot_" + std::to_string(m_robot_id) + "_driver");
    m_state_pub =
        m_node->create_publisher<std_msgs::msg::Float64MultiArray>(TopicForRobot(m_robot_id, "state"), 10);
    m_command_sub = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(
        TopicForRobot(m_robot_id, "vehicle_cmd"),
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnCommand(msg); });
    m_executor->add_node(m_node);

    RCLCPP_INFO(m_node->get_logger(),
                "ROS driver ready: publishing %s, subscribing %s",
                TopicForRobot(m_robot_id, "state").c_str(),
                TopicForRobot(m_robot_id, "vehicle_cmd").c_str());
}

RosControllerDriver::~RosControllerDriver() {
    if (m_node && m_executor)
        m_executor->remove_node(m_node);
}

void RosControllerDriver::ShutdownRos() {
    if (rclcpp::ok())
        rclcpp::shutdown();
}

void RosControllerDriver::OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 3) {
        RCLCPP_WARN(m_node->get_logger(), "Ignoring vehicle_cmd; expected [steering, throttle, brake].");
        return;
    }

    std::lock_guard<std::mutex> lock(m_command_mutex);
    m_steering = std::clamp(msg->data[0], -1.0, 1.0);
    m_throttle = std::clamp(msg->data[1], 0.0, 1.0);
    m_braking = std::clamp(msg->data[2], 0.0, 1.0);
    m_clutch = 0.0;
    m_command_received = true;
}

void RosControllerDriver::Synchronize(double time) {
    m_executor->spin_some();

    {
        std::lock_guard<std::mutex> lock(m_command_mutex);
        if (!m_command_received) {
            m_steering = 0.0;
            m_throttle = 0.0;
            m_braking = 1.0;
            m_clutch = 0.0;
        }
    }

    PublishState(time);
}

void RosControllerDriver::PublishState(double time) {
    if (m_last_state_publish_time >= 0.0 && time - m_last_state_publish_time < m_state_publish_period)
        return;

    const auto pos = m_vehicle.GetPos();
    const auto rot = m_vehicle.GetRot();
    const auto forward = rot.GetAxisX();
    const double yaw = std::atan2(forward.y(), forward.x());

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        pos.x(),
        pos.y(),
        yaw,
        m_vehicle.GetSpeed(),
    };
    m_state_pub->publish(msg);
    m_last_state_publish_time = time;
}

}  // namespace amd_uw
