#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "chrono_vehicle/ChDriver.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amd_uw {

class RosControllerDriver : public chrono::vehicle::ChDriver {
  public:
    RosControllerDriver(chrono::vehicle::ChVehicle& vehicle, int robot_id, double state_publish_rate_hz = 50.0);
    ~RosControllerDriver() override;

    void Synchronize(double time) override;
    void Advance(double step) override {}

    static void ShutdownRos();

  private:
    void OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void PublishState(double time);

    int m_robot_id;
    double m_state_publish_period;
    double m_last_state_publish_time;
    bool m_command_received;

    rclcpp::Node::SharedPtr m_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_command_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_state_pub;

    std::mutex m_command_mutex;
};

}  // namespace amd_uw
