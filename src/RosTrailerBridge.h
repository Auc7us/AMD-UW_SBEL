#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amd_uw {

class RobotRig;

// ROS 2 bridge for one collector's dump bed.
//
// /robot_N/trailer_cmd:    [command]  1 = run one dump cycle, anything else = no-op
// /robot_N/trailer_state:  [state, bed_angle_rad, tailgate_angle_rad]
//
// The dump cycle itself runs in RobotRig at the simulation step rate, not here.
// It has to: the bed and tailgate are angle motors, and holding their motion to a
// bounded slew rate is what keeps the load sliding out instead of being flung, so
// the motion cannot be driven from a 10 Hz controller. A controller therefore only
// asks for a cycle and watches `state` for completion. Repeating the request while
// a cycle is running is harmless.
//
// `state` mirrors RobotRig::DumpState:
//   0 idle, 1 opening gate, 2 tilting, 3 dwell, 4 levelling, 5 closing gate, 6 done
class RosTrailerBridge {
  public:
    RosTrailerBridge(int robot_id, RobotRig& rig);
    ~RosTrailerBridge();

    void Synchronize();

  private:
    void OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void PublishState();

    int m_robot_id;
    RobotRig& m_rig;
    bool m_dump_requested = false;

    rclcpp::Node::SharedPtr m_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_command_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_state_pub;
};

}  // namespace amd_uw
