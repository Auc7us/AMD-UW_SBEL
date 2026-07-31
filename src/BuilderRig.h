#pragma once

#include <memory>
#include <string>

#include "chrono/core/ChCoordsys.h"
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

    // Where on its orbit this builder should wait. The rank's lane rotates each
    // harvest cycle and the builder stays inboard of the collector's drop point.
    void SetStationAngle(double angle_rad);

    // Diagnostics used by the rank-local performance probe.
    const chrono::vehicle::DriverInputs& GetDriverInputs() const { return m_driver_inputs; }
    double GetSpeed() const;
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
    chrono::vehicle::DriverInputs m_driver_inputs;
};

}  // namespace amd_uw
