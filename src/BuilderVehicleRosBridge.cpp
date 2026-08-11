#include "BuilderVehicleRosBridge.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

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

BuilderVehicleRosBridge::BuilderVehicleRosBridge(
    int builder_id,
    chrono::vehicle::ChTrackedVehicle& vehicle)
    : m_builder_id(builder_id), m_vehicle(vehicle) {
    EnsureRosInitialized();

    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_node = rclcpp::Node::make_shared(
        "chrono_builder_" + std::to_string(m_builder_id) + "_driver");
    m_state_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "vehicle_state"), 10);
    m_station_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "station_angle"), 10);
    m_command_sub =
        m_node->create_subscription<std_msgs::msg::Float64MultiArray>(
            TopicForBuilder(m_builder_id, "vehicle_cmd"),
            10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                OnCommand(msg);
            });
    m_executor->add_node(m_node);

    RCLCPP_INFO(
        m_node->get_logger(),
        "Builder driver bridge ready: subscribing %s, publishing %s",
        TopicForBuilder(m_builder_id, "vehicle_cmd").c_str(),
        TopicForBuilder(m_builder_id, "vehicle_state").c_str());
}

BuilderVehicleRosBridge::~BuilderVehicleRosBridge() {
    if (m_node && m_executor)
        m_executor->remove_node(m_node);
}

std::optional<chrono::vehicle::DriverInputs>
BuilderVehicleRosBridge::Synchronize(double time) {
    m_executor->spin_some();
    // Throttle. This used to publish vehicle_state AND station_angle on EVERY sim step
    // -- 2 kHz each at the 5e-4 step -- against controllers that consume them at 10-20 Hz.
    // At 15 builders that plus arm_state was ~90k messages/s, which saturates Fast-DDS:
    // fragments get dropped and subscribers hit "sequence size exceeds remaining buffer"
    // deserialising a truncated Float64MultiArray. Dropped COMMANDS are the real cost.
    // 200 Hz of sim time is ~10 Hz of wall time here, matching the controller rate.
    constexpr double publish_period = 1.0 / 200.0;
    if (m_last_publish_time < 0.0 || time - m_last_publish_time >= publish_period) {
        m_last_publish_time = time;
        PublishState();
    }

    auto command = m_pending_command;
    m_pending_command.reset();
    return command;
}

void BuilderVehicleRosBridge::OnCommand(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != 3) {
        RCLCPP_WARN(
            m_node->get_logger(),
            "Ignoring vehicle_cmd; expected [steering, throttle, braking].");
        return;
    }

    if (!std::all_of(
            msg->data.begin(), msg->data.end(),
            [](double value) { return std::isfinite(value); })) {
        RCLCPP_WARN(
            m_node->get_logger(),
            "Ignoring vehicle_cmd containing a non-finite value.");
        return;
    }

    chrono::vehicle::DriverInputs command;
    command.m_steering = std::clamp(msg->data[0], -1.0, 1.0);
    command.m_throttle = std::clamp(msg->data[1], 0.0, 1.0);
    command.m_braking = std::clamp(msg->data[2], 0.0, 1.0);
    m_pending_command = command;
}

void BuilderVehicleRosBridge::PublishState() {
    const auto pos = m_vehicle.GetChassis()->GetPos();
    const double yaw =
        m_vehicle.GetChassisBody()->GetRot().GetCardanAnglesZYX().z();

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {pos.x(), pos.y(), yaw, m_vehicle.GetSpeed()};
    m_state_pub->publish(msg);

    std_msgs::msg::Float64MultiArray station;
    station.data = {m_station_angle};
    m_station_pub->publish(station);
}

}  // namespace amd_uw
