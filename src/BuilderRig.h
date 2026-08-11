#pragma once

#include <memory>
#include <string>
#include <vector>

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChVector3.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChDriver.h"

namespace chrono {
namespace vehicle {
class ChTrackedVehicle;
namespace m113 {
class M113;
}
}  // namespace vehicle
}  // namespace chrono

namespace amd_uw {

class LrvArm;
#ifdef AMD_UW_ENABLE_ROS2
class BuilderArmRosBridge;
class BuilderVehicleRosBridge;
#endif

// Complete M113 tracked builder and its articulated manipulator.
//
// The builder lives in the rank's ONE shared Chrono system, alongside that
// rank's rover, trailer, rocks, and terrain. That is deliberate and required:
// Chrono only generates contacts between bodies of the same system, so a builder
// in its own system could never touch the rocks it is meant to place. It also
// means there is a single terrain instance per rank rather than a duplicate.
//
// The cost of sharing is that the whole system runs the solver the single-pin
// track needs (see the ChSolverBB setup in main.cpp) -- that is a system-wide
// decision and belongs to the caller, not here.
class BuilderRig {
  public:
    // Diagnostic switch used to bisect the builder's per-step cost.
    struct Options {
        // Skip the manipulator entirely.
        bool with_arm = true;
        // The builder's own build plan: feedstock rock k is laid on wall slot k. Both
        // are indexed by wall slot and are handed straight to BuilderArmRosBridge, which
        // owns the cycle. Empty means the builder has an arm but nothing to build with,
        // which is the pre-existing behaviour.
        std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> pile_rocks;
        std::vector<chrono::ChVector3d> wall_slots;
    };

    BuilderRig(int rank,
               chrono::ChSystem* system,
               const std::string& amd_uw_data_path,
               const chrono::ChCoordsys<>& init_pose,
               const Options& options);
    ~BuilderRig();

    chrono::ChSystem* GetSystem() const;
    chrono::vehicle::ChTrackedVehicle* GetVehicle() const;
    LrvArm* GetArm() const;
    chrono::ChVector3d GetPosition() const;

    void SetDriverInputs(const chrono::vehicle::DriverInputs& inputs);
    // Deliberately no SetChassisFixed: pinning this hull on the heightmap destroys
    // the single-pin track within about a second. See the constructor.
    // Must be called before the shared system is stepped.
    void Synchronize(double time);
    // Advances only this builder's subsystems (powertrain, track assemblies).
    // The shared system itself is stepped once by the rover rig.
    void Advance(double step);

    // Where on its orbit this builder should wait. Published to the orbit controller.
    // The caller derives it from GetPlacedCount(), NOT from the collector's harvest
    // cycle: the builder walks its own course at its own pace and no longer waits on,
    // or is repositioned by, anything its collector does.
    void SetStationAngle(double angle_rad);

    // Rocks this builder has laid on the work circle == the wall slot it is now serving.
    // Zero without an arm bridge, so a builder built with --no_builder-style options
    // simply never advances.
    int GetPlacedCount() const;

    // Sim time before which Synchronize() holds full brake and DISCARDS ROS commands.
    // A single-pin track needs to reach equilibrium on the terrain before anything
    // drives it; station keeping starting mid-settle acts on a track that has not
    // found its wheels yet.
    void SetCommandEnableTime(double time) { m_command_enable_time = time; }

    // Diagnostic: force the builder to never count as parked, which stops the arm ever
    // being offered a pick. Isolates build-cycle cost from drive cost.
    void SetHullParkEnabled(bool enabled) { m_hull_park_enabled = enabled; }

    // Diagnostics used by the rank-local performance probe.
    const chrono::vehicle::DriverInputs& GetDriverInputs() const { return m_driver_inputs; }
    double GetSpeed() const;
    bool IsParked() const { return m_parked; }
    // Largest distance from the chassis reference to any track shoe. A healthy
    // single-pin track stays within roughly half the hull length; a value that
    // keeps growing means shoes have derailed and are being dragged.
    double GetMaxShoeDistance() const;

  private:
    // Destruction is intentionally bridge -> arm -> M113 so every installed
    // object is released before the vehicle it was attached to.
    std::unique_ptr<chrono::vehicle::m113::M113> m_m113;
    std::unique_ptr<LrvArm> m_arm;
#ifdef AMD_UW_ENABLE_ROS2
    std::unique_ptr<BuilderArmRosBridge> m_arm_ros_bridge;
    std::unique_ptr<BuilderVehicleRosBridge> m_vehicle_ros_bridge;
#endif
    // MUST be initialised here. chrono::vehicle::DriverInputs is a plain aggregate
    // with no default member initialisers, so a bare declaration leaves steering,
    // throttle and braking indeterminate -- and Synchronize() feeds them to the M113
    // every step until a ROS command arrives. A builder launched without its
    // controller was therefore driven by whatever was in that memory. Full brake is
    // the right idle state: no controller means do not move.
    chrono::vehicle::DriverInputs m_driver_inputs{/*steering=*/0.0, /*throttle=*/0.0,
                                                  /*braking=*/1.0, /*clutch=*/0.0};
    double m_command_enable_time = 0.0;
    // Braked AND actually stopped, so the arm base frame is steady enough to solve a
    // pose against. NOT a pinned hull -- see the long note in Synchronize.
    bool m_parked = false;
    // Virtual anchor: a spring-damper to the pose the builder was in when it asked to
    // park, applied as a FORCE on the chassis rather than a constraint. A fully braked
    // M113 does not stay put (measured: it creeps at 0.22-0.27 m/s on full brake), and
    // SetFixed -- which does hold it -- tears the single-pin track apart. A force has no
    // velocity discontinuity, so it can hold station without shocking the chain.
    bool m_anchor_active = false;
    chrono::ChVector3d m_anchor_pos;
    double m_anchor_yaw = 0.0;
    unsigned int m_anchor_accumulator = 0;
    bool m_anchor_accumulator_ready = false;
    // Diagnostic escape hatch (--no_build). The arm bridge only offers a pick while the
    // builder is parked, so with this off it drives its lane and never builds.
    bool m_hull_park_enabled = true;
};

}  // namespace amd_uw
