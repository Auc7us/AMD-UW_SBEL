#pragma once

#include <memory>
#include <optional>

#include "chrono_vehicle/ChDriver.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace chrono {
namespace vehicle {
class ChTrackedVehicle;
}
}  // namespace chrono

namespace amd_uw {

// ROS 2 drive bridge for the real tracked builder.
//
// /builder_N/vehicle_cmd:
//   [steering, throttle, braking]
//
// /builder_N/vehicle_state:
//   [x, y, yaw, speed]
class BuilderVehicleRosBridge {
  public:
    BuilderVehicleRosBridge(int builder_id, chrono::vehicle::ChTrackedVehicle& vehicle);
    ~BuilderVehicleRosBridge();

    std::optional<chrono::vehicle::DriverInputs> Synchronize();

  private:
    void OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void PublishState();

    int m_builder_id;
    chrono::vehicle::ChTrackedVehicle& m_vehicle;
    rclcpp::Node::SharedPtr m_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_command_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_state_pub;
    std::optional<chrono::vehicle::DriverInputs> m_pending_command;
};

}  // namespace amd_uw
