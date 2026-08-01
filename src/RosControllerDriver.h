#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono_vehicle/ChDriver.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amd_uw {

class RosControllerDriver : public chrono::vehicle::ChDriver {
  public:
    RosControllerDriver(chrono::vehicle::ChVehicle& vehicle,
                        int robot_id,
                        const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& rocks,
                        const chrono::ChVector3d& home_position);
    ~RosControllerDriver() override;

    void Synchronize(double time) override;
    void Advance(double step) override {}

    static void ShutdownRos();

  private:
    void OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void PublishTelemetry(double time);

  public:
    // The drop point moves each harvest cycle, so home is not fixed at construction.
    // Published on /robot_N/homePos, which the drive controller already re-reads every
    // message, so a change propagates without any handshake.
    void SetHomePosition(const chrono::ChVector3d& home) { m_home_position = home; }

  private:

    int m_robot_id;
    const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& m_rocks;
    bool m_command_received;
    bool m_no_command_warned = false;
    bool m_multi_publisher_warned = false;
    double m_dup_check_last_time = -1.0;
    int m_dup_confirmations = 0;

    // Throttle rise-rate limiter: the commanded throttle (m_throttle_cmd) is
    // applied to m_throttle no faster than m_throttle_rise_per_s per second, so a
    // step full-throttle command ramps in gently instead of wheelie-ing / bouncing
    // the front wheels. Throttle release is applied immediately (braking stays
    // responsive). Full throttle takes ~1/m_throttle_rise_per_s seconds to reach.
    // Steering is slewed, not stepped: the rack-pinion actuator takes the input as a
    // commanded POSITION, so a jump is an instantaneous velocity into the suspension.
    double m_steering_cmd = 0.0;
    double m_steering_rate_per_s = 2.5;  // full lock in ~0.8 s
    double m_throttle_cmd = 0.0;
    double m_throttle_rise_per_s = 0.35;  // ~2.9 s from 0 to full
    double m_last_sync_time = -1.0;
    // Telemetry publish throttles; see PublishTelemetry.
    double m_last_ego_pub_time = -1.0;
    double m_last_static_pub_time = -1.0;

    rclcpp::Node::SharedPtr m_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_command_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_ego_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_target_pos_pub;
    // Spawn pose, so the drive controller can return here at end of mission
    // instead of duplicating the site-layout maths in Python.
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_home_pos_pub;
    chrono::ChVector3d m_home_position;

    std::mutex m_command_mutex;
};

}  // namespace amd_uw
