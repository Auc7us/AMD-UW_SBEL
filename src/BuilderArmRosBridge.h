#pragma once

#include <array>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "chrono/core/ChVector3.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amd_uw {

class LrvArm;

// ROS 2 bridge for the tracked builder arm, and the owner of that builder's BUILD
// CYCLE -- pick a rock out of the feedstock heap, lay it on the next slot of the work
// circle, repeat.
//
// The cycle is the builder's own. Nothing here reads the collector's harvest state, the
// harvest cycle index, or any lane rotation; slot k's rock, the slot's world point, and
// the orbit angle the hull must hold to serve it all come from RobotLayout's build plan,
// indexed only by how many rocks THIS builder has already laid.
//
// Topics
//   in  /builder_N/arm_cmd
//         5  elements: [theta1..4, finger_closure_m] -- direct actuator command, kept
//                      for the deterministic actuation test. Bypasses the build cycle.
//         12 elements: [seq, target_index, rock_x, rock_y, grab_theta1..4,
//                      place_theta1..4] -- a pick-and-place, same shape as the rover's.
//   out /builder_N/arm_base_pose   [x, y, z, qw, qx, qy, qz]   IK frame origin
//   out /builder_N/pick_target     [ready, index, x, y, z]     next rock (privileged)
//   out /builder_N/place_target    [x, y, z]                   next wall slot
//   out /builder_N/arm_status      [seq, state, index, success, error_code, sim_time,
//                                  slot, usable_rocks, slot_angle_rad]
//                                 The last three are appended for the collector's
//                                 return leg: the slot being consumed, how many
//                                 unlaid rocks this builder still owns, and the
//                                 angle of that slot about the site centre.
//   out /builder_N/arm_state       [theta1..4, finger1, finger2, ee_x, ee_y, ee_z]
//
// PRIVILEGED INFORMATION IS DELIBERATE. pick_target is read straight off the rock body
// rather than perceived: this is a mimic of the construction task, not a perception
// demo, and the arm has to hit a 0.2-scale rock with a gripper whose jaws open 0.388 m.
class BuilderArmRosBridge {
  public:
    // `seed_rocks` is the one heap the builder starts with; `wall_slots[k]` is where the
    // k-th rock laid goes. They are NOT indexed together -- the seed heap runs out long
    // before the course does, and everything after it comes from SetDeliveredRockSource.
    //
    // Rocks are FIXED at rest. This class unfixes nothing itself (LrvArm does that when
    // its gripper locks on) but it re-fixes each one once it has been laid, so at most
    // the one in the gripper is ever a dynamic body.
    BuilderArmRosBridge(int builder_id,
                        LrvArm& arm,
                        std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> seed_rocks,
                        std::vector<chrono::ChVector3d> wall_slots);
    ~BuilderArmRosBridge();

    // Rocks the collector has delivered since the last call, appended to whatever the
    // builder can already see. Called every publish, so a load that lands mid-run is
    // picked up without anything having to signal it.
    using DeliveredRockSource = std::function<std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>()>;
    void SetDeliveredRockSource(DeliveredRockSource source) { m_delivered_source = std::move(source); }

    // apply_commands=false spins the executor and publishes state but DISCARDS any
    // command, so the arm holds its pose. Used during the builder's settle window: a
    // 2x-scaled arm slewing a 4.7 rad swing while the hull is still finding equilibrium
    // is exactly the load the settle window exists to avoid.
    void Synchronize(double time, bool apply_commands = true);

    // The hull is parked (pinned) and may be worked from. Set by BuilderRig each step.
    // A pick is never offered or accepted while the builder is driving: the arm base
    // would be moving under a solved pose, and the rock would be laid on the roll.
    void SetHullParked(bool parked) { m_hull_parked = parked; }

    // Delete feedstock the builder has driven past. See the definition: a rock behind the
    // builder can never be laid, and this is a one-way orbit.
    void ClearStrandedFeedstock(double time);

    // Rocks laid so far == the next wall slot == how far round the lane the hull should
    // now be. BuilderRig turns this into the station angle it publishes.
    int GetPlacedCount() const { return m_placed_count; }
    // Angular nudge the builder is asking for so a rock lying just outside its envelope
    // comes into it. Added to the station angle by BuilderRig; zero unless the arm is
    // starved with a near miss on the ground. See the starved branch of PublishBuildTopics.
    double GetStationFetchOffset() const { return m_station_fetch_offset; }
    bool BuildComplete() const { return m_placed_count >= static_cast<int>(m_wall_slots.size()); }

  private:
    struct DirectCommand {
        std::array<double, 4> theta = {0.0, 0.0, 0.0, 0.0};
        double finger_closure = 0.0;
    };

    struct PickPlaceCommand {
        double command_seq = 0.0;
        int target_index = -1;
        // World x/y of the rock this command was solved against. Authoritative for
        // WHICH rock it means -- see FindFeedstockNear.
        double rock_x = 0.0;
        double rock_y = 0.0;
        std::array<double, 4> grab_theta = {0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> place_theta = {0.0, 0.0, 0.0, 0.0};
    };

    void OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void PublishStateThrottled(double time);
    void PublishState();
    void PublishBuildTopics(double time);
    // Slot ready to be worked, or -1. Requires a parked hull, an unlaid slot, and a rock
    // in reach.
    int ReadySlot() const;
    // Fold whatever the collector has delivered into the feedstock pool, then choose the
    // nearest un-consumed rock inside the arm's envelope. Sets m_selected.
    void UpdateFeedstock(double time);
    // The un-consumed feedstock rock nearest (x, y), or null if none is within
    // command_rock_match_tol. Used to identify the rock a pick/place command was
    // actually solved for; see the call site.
    std::shared_ptr<chrono::ChBodyAuxRef> FindFeedstockNear(double x, double y) const;

    int m_builder_id;
    LrvArm& m_arm;
    // Everything this builder may pick from: the seed heap, then each delivered load.
    // Grows; never shrinks. m_consumed is what has already been laid (or failed and been
    // written off), keyed by raw pointer because the pool holds the owning references.
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_feedstock;
    std::unordered_set<const chrono::ChBodyAuxRef*> m_consumed;
    // Failed grabs per rock. A rock is only written off for good once this reaches
    // max_grab_attempts; until then a failure puts it back in the pile.
    std::unordered_map<const chrono::ChBodyAuxRef*, int> m_grab_attempts;
    DeliveredRockSource m_delivered_source;
    std::shared_ptr<chrono::ChBodyAuxRef> m_selected;  // rock currently on offer
    double m_last_feed_refresh = -1.0;
    std::vector<chrono::ChVector3d> m_wall_slots;

    int m_placed_count = 0;
    // Latched for the slot it was taken for: the moment a fetched rock enters the
    // envelope the builder stops being starved, and recomputing the offset then would
    // snap the station back and take the rock straight out of reach again.
    double m_station_fetch_offset = 0.0;
    int m_fetch_offset_slot = -1;
    int m_stranded_count = 0;
    double m_last_status_report = -1.0;
    bool m_hull_parked = false;
    double m_last_started_seq = -1.0;
    double m_settled_seq = -2.0;  // command_seq whose completion was already booked
    int m_active_slot = -1;
    std::shared_ptr<chrono::ChBodyAuxRef> m_active_rock;
    double m_last_time = 0.0;
    bool m_reported_complete = false;
    // Diagnostics for a builder that is parked with nothing in reach.
    double m_starved_since = -1.0;
    // When the builder first found itself parked unable to reach its own wall slot.
    // See unservable_slot_timeout.
    double m_unservable_since = -1.0;
    double m_last_starved_report = -1.0e9;

    rclcpp::Node::SharedPtr m_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_command_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_base_pose_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pick_target_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_place_target_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_status_pub;

    std::mutex m_command_mutex;
    std::optional<DirectCommand> m_pending_direct;
    std::optional<PickPlaceCommand> m_pending_pick_place;

    double m_last_publish_time = -1.0;
    double m_last_build_pub_time = -1.0;
};

}  // namespace amd_uw
