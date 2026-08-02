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

// Duplicate-publisher detection: ignore the first seconds (DDS discovery, plus
// phantom publishers left behind by hard-killed nodes) and require the count to stay
// high across several spaced samples before believing it.
constexpr double dup_check_start_time = 3.0;
constexpr double dup_check_interval = 1.0;
constexpr int dup_check_confirmations = 3;

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
    // Store steering as a TARGET, slewed toward in Synchronize. It must never be
    // applied as a step -- see the rate limiter there.
    m_steering_cmd = std::clamp(msg->data[0], -1.0, 1.0);
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
    //
    // Sampled over several seconds, not on the first reading: a pkill -9 leaves the
    // dead node's publisher in DDS discovery for a lease period, so a clean graph
    // looks duplicated for the first moments of the very next run.
    if (!m_multi_publisher_warned && time > dup_check_start_time &&
        time - m_dup_check_last_time >= dup_check_interval) {
        m_dup_check_last_time = time;
        if (m_command_sub->get_publisher_count() > 1) {
            if (++m_dup_confirmations >= dup_check_confirmations) {
                m_multi_publisher_warned = true;
                RCLCPP_WARN(m_node->get_logger(),
                            "%zu publishers on %s for %.0f s -- expected 1. They fight each other: commands "
                            "alternate, so braking flickers 0/1, throttle never sticks, and the rover sits "
                            "still. Check for a leftover controller AND for a second sim on this ROS graph: "
                            "pgrep -af 'pure_pursuit_controller|demo_SYN'",
                            m_command_sub->get_publisher_count(),
                            TopicForRobot(m_robot_id, "vehicle_cmd").c_str(),
                            dup_check_confirmations * dup_check_interval);
            }
        } else {
            m_dup_confirmations = 0;
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_command_mutex);
        if (!m_command_received) {
            m_steering_cmd = 0.0;
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

        const double dt = (m_last_sync_time >= 0.0) ? (time - m_last_sync_time) : 0.0;
        m_last_sync_time = time;

        // Rate-limit STEERING, in the sim's own time, and never apply it as a step.
        //
        // Chrono's rack-pinion steering is a ChLinkLockLinActuator driven by a
        // ChFunctionConst: ChRackPinion::Synchronize turns the steering input straight
        // into the actuator's commanded POSITION. Re-setting that constant is therefore
        // a position discontinuity -- an instantaneous velocity in the steering linkage,
        // which drives an impulse into the front suspension. It is the same hazard the
        // trailer bed has, and the bed was given a bounded slew rate for exactly this
        // reason; the steering never was.
        //
        // A controller publishing at 20 Hz delivers a staircase, so this fired ~20 times
        // a second even in steady cruise, and harder still on a mode switch (target
        // following -> return home) or when a different publisher takes over. Slewing
        // here, at the 5e-4 s physics step, turns that staircase into a continuous ramp.
        // Rate-limiting in the Python controller is not enough: it only smooths one
        // publisher's own output and cannot smooth the jump BETWEEN publishers.
        if (dt > 0.0) {
            const double max_step = m_steering_rate_per_s * dt;
            m_steering = std::clamp(m_steering_cmd, m_steering - max_step, m_steering + max_step);
        }

        // Rate-limit the throttle RISE (ramp up gently); apply any decrease
        // immediately so lift-off / coasting stays responsive.
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

    // Element 5 is SIMULATION time, and the controller needs it badly.
    //
    // Every deadline in pure_pursuit_controller is measured with the ROS clock,
    // which here is wall time, while the thing being timed happens in sim time --
    // and this sim runs ~20x slower than real time. So a "10 s" stop timeout is
    // really 0.5 s of sim (which is why nearly every grab in every run so far
    // logged `TIMEOUT, dwell=0.00s`, having never had time to settle), and a "15 s"
    // manoeuvring allowance is 0.75 s of sim. Worse, the scale factor is not a
    // constant: it moves with rank count, terrain and machine load, so a wall-clock
    // number cannot express a physical deadline at all. The manipulator already
    // learned this the hard way and grew command_timeout_sim_s; the drive
    // controller had no sim clock to use. Now it does.
    std_msgs::msg::Float64MultiArray ego_state;
    ego_state.data = {
        pos.x(),
        pos.y(),
        yaw,
        m_vehicle.GetSpeed(),
        pos.z(),
        time,
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
