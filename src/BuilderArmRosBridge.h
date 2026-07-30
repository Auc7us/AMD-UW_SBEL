#pragma once

#include <array>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amd_uw {

class LrvArm;

// Direct ROS 2 actuator bridge for the tracked builder arm.
//
// /builder_N/arm_cmd:
//   [theta1, theta2, theta3, theta4, finger_closure_m]
//
// /builder_N/arm_state:
//   [theta1, theta2, theta3, theta4,
//    finger1_m, finger2_m, end_effector_x, end_effector_y, end_effector_z]
class BuilderArmRosBridge {
  public:
    BuilderArmRosBridge(int builder_id, LrvArm& arm);
    ~BuilderArmRosBridge();

    void Synchronize();

  private:
    struct Command {
        std::array<double, 4> theta = {0.0, 0.0, 0.0, 0.0};
        double finger_closure = 0.0;
    };

    void OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void PublishState();

    int m_builder_id;
    LrvArm& m_arm;
    rclcpp::Node::SharedPtr m_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_command_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_state_pub;
    std::optional<Command> m_pending_command;
};

}  // namespace amd_uw
