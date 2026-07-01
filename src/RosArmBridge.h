#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "LrvArm.h"

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amd_uw {

class RosArmBridge {
  public:
    RosArmBridge(int robot_id,
                 LrvArm& arm,
                 const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& rocks,
                 const std::vector<double>& rock_top_heights,
                 std::shared_ptr<chrono::vehicle::WheeledTrailer> trailer,
                 double height_probe_z);
    ~RosArmBridge();

    void Synchronize(double time, chrono::vehicle::RigidTerrain& terrain);

    // Rocks welded to the trailer bed (collision managed here). The rig must not
    // re-toggle their collision in its distance-based activation.
    const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& WeldedRocks() const { return m_welded_rocks; }

  private:
    struct ArmCommand {
        double command_seq = 0.0;
        int target_index = -1;
        double rock_x = 0.0;
        double rock_y = 0.0;
    };

    void OnArmCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void PublishStatus();
    chrono::ChVector3d PlacePoint(int slot) const;

    int m_robot_id;
    LrvArm& m_arm;
    const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& m_rocks;
    const std::vector<double>& m_rock_top_heights;
    std::shared_ptr<chrono::vehicle::WheeledTrailer> m_trailer;
    double m_height_probe_z;
    double m_last_started_seq = -1.0;
    int m_place_count = 0;  // rocks dispatched to placement so far (grid slot index)
    std::shared_ptr<chrono::ChBodyAuxRef> m_inflight_rock;     // rock of the active pick/place
    double m_welded_seq = -2.0;                                // command_seq already welded
    std::vector<std::shared_ptr<chrono::ChLinkLockLock>> m_bed_welds;  // placed rocks welded to trailer
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_welded_rocks;  // rocks welded to the bed

    rclcpp::Node::SharedPtr m_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_arm_cmd_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_arm_status_pub;

    std::mutex m_command_mutex;
    std::optional<ArmCommand> m_pending_command;
};

}  // namespace amd_uw
