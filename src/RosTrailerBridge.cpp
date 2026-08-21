#include "RosTrailerBridge.h"

#include <algorithm>
#include <cmath>

#include "RobotLayout.h"
#include "RobotRig.h"

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

RosTrailerBridge::RosTrailerBridge(int robot_id, RobotRig& rig) : m_robot_id(robot_id), m_rig(rig) {
    EnsureRosInitialized();

    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_node = rclcpp::Node::make_shared("chrono_robot_" + std::to_string(m_robot_id) + "_trailer");
    m_state_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForRobot(m_robot_id, "trailer_state"), 10);
    m_command_sub = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(
        TopicForRobot(m_robot_id, "trailer_cmd"),
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnCommand(msg); });
    m_executor->add_node(m_node);

    RCLCPP_INFO(m_node->get_logger(),
                "Trailer bridge ready: subscribing %s, publishing %s",
                TopicForRobot(m_robot_id, "trailer_cmd").c_str(),
                TopicForRobot(m_robot_id, "trailer_state").c_str());
}

RosTrailerBridge::~RosTrailerBridge() {
    if (m_node && m_executor)
        m_executor->remove_node(m_node);
}

void RosTrailerBridge::Synchronize() {
    m_executor->spin_some();

    if (m_dump_requested) {
        m_dump_requested = false;
        if (m_rig.RequestTrailerDump())
            RCLCPP_INFO(m_node->get_logger(), "Dump cycle started for robot %d.", m_robot_id);
    }

    PublishState();
}

void RosTrailerBridge::OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_WARN(m_node->get_logger(), "Ignoring trailer_cmd; expected at least [command].");
        return;
    }
    if (!std::isfinite(msg->data[0])) {
        RCLCPP_WARN(m_node->get_logger(), "Ignoring trailer_cmd containing a non-finite value.");
        return;
    }

    // Only a dump request is defined. The cycle always returns the bed to level and
    // the gate to closed on its own, so there is no separate reset command.
    if (static_cast<int>(std::lround(msg->data[0])) == 1)
        m_dump_requested = true;
}

void RosTrailerBridge::PublishState() {
    // World position of the trailer's rear axle, averaged over that axle's spindles.
    //
    // APPENDED so existing readers of [state, bed, tailgate] are unaffected. It exists
    // because the load leaves from the BED, not from the tractor: the drop band was being
    // judged at the tractor's own reference point, which parks the trailer short of the
    // drop point -- and short, on a clockwise run-in, means the rocks land on the far side
    // from the builder and roll away from it. The controller needs a reference that is
    // actually where the rocks come out, and the articulation angle makes the tractor pose
    // plus a fixed offset the wrong answer on a curve.
    double axle_x = 0.0;
    double axle_y = 0.0;
    bool have_axle = false;
    if (auto trailer = m_rig.GetTrailer()) {
        const auto& axles = trailer->GetAxles();
        if (!axles.empty()) {
            // Rear axle: the last one, which is the one the bed discharges over.
            const auto& axle = axles.back();
            int n = 0;
            for (const auto& wheel : axle->GetWheels()) {
                if (!wheel || !wheel->GetSpindle())
                    continue;
                const auto pos = wheel->GetSpindle()->GetPos();
                axle_x += pos.x();
                axle_y += pos.y();
                ++n;
            }
            if (n > 0) {
                axle_x /= n;
                axle_y /= n;
                have_axle = true;
            }
        }
    }

    // World position of the REAR GATE EDGE CENTRE -- the bottom hinge of the tailgate,
    // which is the lip the load actually pours over.
    //
    // This is a better drop reference than the axle for the reason the bed hinge exists
    // at all (see RobotRig::BuildTrailerBed): the bed is hinged ON the rear lip precisely
    // so that lip stays put in space while the tub rises, so the pour line is fixed here
    // and nowhere else. The axle is roughly a metre forward of it, and judging arrival
    // there drops the load a metre short -- some rocks inside the builder's pickup radius,
    // some outside, which is exactly the scatter being seen.
    //
    // Local offset matches tailgate_hinge_local in RobotRig: half the floor length aft,
    // plus half a wall thickness, raised by half a thickness. Taken against the BED's live
    // pose rather than the chassis, so it stays correct while the bed is tilted.
    double gate_x = 0.0;
    double gate_y = 0.0;
    bool have_gate = false;
    if (auto bed = m_rig.GetTrailerBed()) {
        const double t = trailer_bed_thickness;
        const chrono::ChVector3d gate_local(-(trailer_bed_half_x + t / 2.0), 0.0, t / 2.0);
        const auto gate = bed->GetPos() + bed->GetRot().Rotate(gate_local);
        gate_x = gate.x();
        gate_y = gate.y();
        have_gate = true;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {static_cast<double>(static_cast<int>(m_rig.GetDumpState())),
                m_rig.GetBedAngle(),
                m_rig.GetTailgateAngle(),
                // valid flag first, so a consumer never has to guess whether (0, 0) is
                // the origin or a missing trailer
                have_axle ? 1.0 : 0.0,
                axle_x,
                axle_y,
                have_gate ? 1.0 : 0.0,
                gate_x,
                gate_y};
    m_state_pub->publish(msg);
}

}  // namespace amd_uw
