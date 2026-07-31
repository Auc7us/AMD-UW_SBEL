#include "RosControllerDriver.h"

#include <algorithm>
#include <cmath>

#include "chrono/core/ChQuaternion.h"
#include "std_msgs/msg/multi_array_dimension.hpp"

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
                                         const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& rocks,
                                         const chrono::ChVector3d& home_position)
    : chrono::vehicle::ChDriver(vehicle),
      m_robot_id(robot_id),
      m_rocks(rocks),
      m_command_received(false),
      m_home_position(home_position) {
    EnsureRosInitialized();

    m_steering = 0.0;
    m_throttle = 0.0;
    m_braking = 1.0;
    m_clutch = 0.0;

    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_node = rclcpp::Node::make_shared("chrono_robot_" + std::to_string(m_robot_id) + "_driver");
    m_ego_state_pub =
        m_node->create_publisher<std_msgs::msg::Float64MultiArray>(TopicForRobot(m_robot_id, "egoState"), 10);
    m_target_pos_pub =
        m_node->create_publisher<std_msgs::msg::Float64MultiArray>(TopicForRobot(m_robot_id, "targetPos"), 10);
    m_home_pos_pub =
        m_node->create_publisher<std_msgs::msg::Float64MultiArray>(TopicForRobot(m_robot_id, "homePos"), 10);
    m_command_sub = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(
        TopicForRobot(m_robot_id, "vehicle_cmd"),
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnCommand(msg); });
    m_executor->add_node(m_node);

    RCLCPP_INFO(m_node->get_logger(),
                "ROS driver ready: publishing %s and %s, subscribing %s",
                TopicForRobot(m_robot_id, "egoState").c_str(),
                TopicForRobot(m_robot_id, "targetPos").c_str(),
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
    // Store commanded throttle as a target; the applied m_throttle is ramped
    // toward it in Synchronize() so a step full-throttle command doesn't slam in.
    m_throttle_cmd = std::clamp(msg->data[1], 0.0, 1.0);
    m_braking = std::clamp(msg->data[2], 0.0, 1.0);
    m_clutch = 0.0;
    m_command_received = true;
}

void RosControllerDriver::Synchronize(double time) {
    m_executor->spin_some();

    // Two controllers on one command topic fight each other: the commands alternate,
    // so braking flickers between 0 and 1, throttle never sticks, and a stale node
    // left over from an earlier run can drive the arm at rock positions that no
    // longer exist. It looks like a broken vehicle, not a duplicated controller, so
    // say so explicitly.
    if (!m_multi_publisher_warned && m_command_sub->get_publisher_count() > 1) {
        m_multi_publisher_warned = true;
        RCLCPP_WARN(m_node->get_logger(),
                    "%zu publishers on %s -- expected 1. A leftover controller from a previous run will "
                    "fight this one (flickering brake, no throttle, stale targets). Kill strays with: "
                    "pkill -f pure_pursuit_controller",
                    m_command_sub->get_publisher_count(), TopicForRobot(m_robot_id, "vehicle_cmd").c_str());
    }

    {
        std::lock_guard<std::mutex> lock(m_command_mutex);
        if (!m_command_received) {
            m_steering = 0.0;
            m_throttle_cmd = 0.0;
            m_braking = 1.0;
            m_clutch = 0.0;
            // A rank with no controller sits still and says nothing, which reads
            // exactly like a stuck or broken robot. Say it out loud once, because
            // the usual cause is a robot_ids list that does not cover every rank
            // (it defaults to "1", so a 4-robot run needs robot_ids:=1,2,3,4).
            if (!m_no_command_warned && time > 5.0) {
                m_no_command_warned = true;
                RCLCPP_WARN(m_node->get_logger(),
                            "No command on %s after %.1f s of simulation; robot %d is parked. "
                            "Is a controller running for it?",
                            TopicForRobot(m_robot_id, "vehicle_cmd").c_str(), time, m_robot_id);
            }
        }

        // Rate-limit the throttle RISE (ramp up gently); apply any decrease
        // immediately so lift-off / coasting stays responsive.
        const double dt = (m_last_sync_time >= 0.0) ? (time - m_last_sync_time) : 0.0;
        m_last_sync_time = time;
        if (m_throttle_cmd > m_throttle) {
            const double max_rise = m_throttle_rise_per_s * std::max(dt, 0.0);
            m_throttle = std::min(m_throttle_cmd, m_throttle + max_rise);
        } else {
            m_throttle = m_throttle_cmd;
        }
    }

    PublishTelemetry(time);
}

void RosControllerDriver::PublishTelemetry(double time) {
    // Every consumer of these topics is a 10 Hz Python controller, and this used to
    // publish all three on every 5e-4 s physics step -- 2 kHz of sim time, and
    // target_pos carries the whole rock list each time. Rate-limit to something a
    // Python node can actually drain, so what it reads is the newest sample rather
    // than whatever is at the head of a permanently full queue.
    constexpr double ego_period = 0.02;    // 50 Hz of sim time
    constexpr double static_period = 0.5;  // rock list and home point move rarely
    const bool publish_ego = m_last_ego_pub_time < 0.0 || time - m_last_ego_pub_time >= ego_period;
    const bool publish_static = m_last_static_pub_time < 0.0 || time - m_last_static_pub_time >= static_period;
    if (!publish_ego && !publish_static)
        return;

    const auto pos = m_vehicle.GetPos();
    const auto rot = m_vehicle.GetRot();
    const auto forward = rot.GetAxisX();
    const double yaw = std::atan2(forward.y(), forward.x());

    std_msgs::msg::Float64MultiArray ego_state;
    ego_state.data = {
        pos.x(),
        pos.y(),
        yaw,
        m_vehicle.GetSpeed(),
        pos.z(),
    };
    if (publish_ego) {
        m_last_ego_pub_time = time;
        m_ego_state_pub->publish(ego_state);
    }
    if (!publish_static)
        return;
    m_last_static_pub_time = time;

    std_msgs::msg::Float64MultiArray target_pos;
    target_pos.layout.dim.resize(1);
    target_pos.layout.dim[0].label = "xyz";
    target_pos.layout.dim[0].size = m_rocks.size();
    target_pos.layout.dim[0].stride = 3 * m_rocks.size();
    target_pos.data.reserve(3 * m_rocks.size());
    for (const auto& rock : m_rocks) {
        const auto rock_pos = rock->GetPos();
        target_pos.data.push_back(rock_pos.x());
        target_pos.data.push_back(rock_pos.y());
        target_pos.data.push_back(rock_pos.z());
    }
    m_target_pos_pub->publish(target_pos);

    std_msgs::msg::Float64MultiArray home_pos;
    home_pos.data = {m_home_position.x(), m_home_position.y(), m_home_position.z()};
    m_home_pos_pub->publish(home_pos);
}

}  // namespace amd_uw
