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
//
// /builder_N/station_angle:
//   [angle_rad] -- where on its orbit this builder should be waiting. The rank's
//   lane rotates one step every harvest cycle, and the builder is meant to stay
//   radially inboard of the collector's current drop point, so its station moves
//   with it. The orbit controller drives round to this angle and holds there.
class BuilderVehicleRosBridge {
  public:
    BuilderVehicleRosBridge(int builder_id, chrono::vehicle::ChTrackedVehicle& vehicle);
    ~BuilderVehicleRosBridge();

    // `time` is SIM time, used only to throttle publishing. See publish_period.
    std::optional<chrono::vehicle::DriverInputs> Synchronize(double time);

    // Set by the owning rank when its harvest cycle advances.
    void SetStationAngle(double angle_rad) { m_station_angle = angle_rad; }

  private:
    void OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void PublishState();

    int m_builder_id;
    chrono::vehicle::ChTrackedVehicle& m_vehicle;
    rclcpp::Node::SharedPtr m_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_command_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_station_pub;
    double m_station_angle = 0.0;
    double m_last_publish_time = -1.0;
    std::optional<chrono::vehicle::DriverInputs> m_pending_command;
};

}  // namespace amd_uw
