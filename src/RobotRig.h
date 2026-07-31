#pragma once

#include <memory>
#include <string>
#include <vector>

#include "RockField.h"

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

namespace amd_uw {

class DriverWrapper;
class LrvArm;
#ifdef AMD_UW_ENABLE_ROS2
class RosArmBridge;
class RosTrailerBridge;
#endif

class RobotRig {
  public:
    RobotRig(chrono::ChContactMethod contact_method,
             int rank,
             int robot_index,
             int num_robots,
             double tire_step_size,
             double render_step_size);
    ~RobotRig();

    chrono::ChSystem* GetSystem() const;
    chrono::vehicle::WheeledVehicle* GetVehicle() const;
    int GetRobotIndex() const { return m_robot_index; }
    std::shared_ptr<chrono::vehicle::WheeledTrailer> GetTrailer() const;
    chrono::vehicle::ChDriver* GetDriver() const;
    const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& GetRocks() const;

    void InitializeOnTerrain(chrono::vehicle::ChTerrain& terrain,
                             const std::shared_ptr<chrono::ChContactMaterial>& rock_mat,
                             const std::string& chrono_data_path,
                             const std::string& amd_uw_data_path,
                             double height_probe_z,
                             double vehicle_start_clearance,
                             double seat_clearance,
                             double settle_time,
                             double step_size,
                             const RockFieldConfig& rock_field_config);

    void Synchronize(double time, chrono::vehicle::ChTerrain& terrain);
    void Advance(double step);
    chrono::vehicle::DriverInputs GetDriverInputs() const;

    // Last commanded inputs as the controller sent them, and as the traction guard
    // left them. Reported by the perf probe so a rover that will not accelerate can
    // be traced to either the controller or the guard.
    const chrono::vehicle::DriverInputs& GetRawDriverInputs() const { return m_last_raw_inputs; }
    const chrono::vehicle::DriverInputs& GetGuardedDriverInputs() const { return m_last_guarded_inputs; }
    // Fraction of recent steps in which the guard was over its lateral limit.
    double GetGuardLimitFraction() const {
        return (m_guard_steps > 0) ? static_cast<double>(m_guard_limited_steps) / m_guard_steps : 0.0;
    }
    void ResetGuardStats() { m_guard_steps = m_guard_limited_steps = 0; }
    double GetSpeed() const { return m_vehicle->GetSpeed(); }

    // Trailer dump cycle. The stages run in order and each one finishes before the
    // next starts, so the tailgate is never fighting the tilting bed.
    enum class DumpState { IDLE, OPENING_GATE, TILTING, DWELL, LEVELING, CLOSING_GATE, DONE };

    // Request one dump cycle. Ignored unless idle or already finished, so a
    // controller republishing the request at its control rate cannot restart the
    // cycle midway. Returns whether the request started a cycle.
    bool RequestTrailerDump();
    DumpState GetDumpState() const { return m_dump_state; }
    // Current commanded angles, radians. Bed angle is the tilt about the trailer
    // lateral axis; gate angle is 0 closed.
    double GetBedAngle() const { return m_bed_angle; }
    double GetTailgateAngle() const { return m_tailgate_angle; }
    void LogMotionIfNeeded(int step_number,
                           int motion_log_steps,
                           chrono::vehicle::ChTerrain& terrain) const;

  private:
    void InitializeVehicle(const chrono::ChCoordsys<>& init_pos);
    void InitializeTrailer();
    void ReseatRig(chrono::vehicle::ChTerrain& terrain,
                   const std::vector<chrono::ChBody*>& preexisting_bodies,
                   double height_probe_z,
                   double seat_clearance);
    void InitializeArm(const std::string& amd_uw_data_path);
    void InitializeTrailerBed();
    void DumpTrailerBed();
    void InitializeDriver();
#ifdef AMD_UW_ENABLE_ROS2
    void InitializeArmBridge(double height_probe_z);
#endif
    void Settle(chrono::vehicle::ChTerrain& terrain, double settle_time, double step_size);
    void UpdateRockCollisionActivation();
    // Watches each trailer wheel for the two things that can launch one wheel out
    // of nowhere: a discontinuous terrain-height query under it, and a tire
    // vertical force far above its own settled load. See the definition.
    void CheckTrailerWheelAnomalies(double time, chrono::vehicle::ChTerrain& terrain);

    // Friction-circle traction guard: clamps the driver's (steering, throttle,
    // braking) to what the front tires can actually deliver, so the rover slows to
    // turn (instead of plowing straight past) and doesn't accelerate out of grip.
    // Load-aware via the measured front-axle normal force; open-loop (mu*g) when the
    // static reference isn't available.
    void ApplyTractionGuard(chrono::vehicle::DriverInputs& inputs, chrono::vehicle::ChTerrain& terrain) const;
    double FrontAxleNormalLoad(chrono::vehicle::ChTerrain& terrain) const;

    int m_rank;
    int m_robot_index;
    int m_num_robots;
    chrono::ChContactMethod m_contact_method;
    double m_tire_step_size;
    double m_render_step_size;

    std::unique_ptr<chrono::vehicle::WheeledVehicle> m_vehicle;
    std::shared_ptr<chrono::vehicle::WheeledTrailer> m_trailer;
    std::shared_ptr<chrono::ChBody> m_trailer_bed;
    std::shared_ptr<chrono::ChBody> m_trailer_tailgate;
    std::shared_ptr<chrono::ChLinkMotorRotationAngle> m_trailer_tailgate_hinge;
    std::shared_ptr<chrono::ChLinkMotorRotationAngle> m_trailer_bed_motor;

    // Dump cycle state. Both motors are angle motors, so their commanded angle is
    // slewed toward its target at a bounded rate every step rather than stepped:
    // re-setting an angle motor's constant is a position discontinuity, i.e. an
    // instantaneous velocity, which throws the load out of the bed instead of
    // letting it slide.
    DumpState m_dump_state = DumpState::IDLE;
    double m_bed_angle = 0.0;
    double m_tailgate_angle = 0.0;
    double m_dump_stage_time = 0.0;
    double m_last_dump_time = -1.0;
    void AdvanceDumpCycle(double time);
    void ReportDumpOutcome(double time);
    void CheckWheelSinkage(double time, chrono::vehicle::ChTerrain& terrain);
    void CheckStuck(double time, chrono::vehicle::ChTerrain& terrain);
    bool RockIsInBed(const std::shared_ptr<chrono::ChBodyAuxRef>& rock) const;
    std::unique_ptr<chrono::vehicle::ChDriver> m_driver;
    std::shared_ptr<chrono::vehicle::ChInteractiveDriver> m_irr_driver;
    std::unique_ptr<LrvArm> m_arm;
#ifdef AMD_UW_ENABLE_ROS2
    std::unique_ptr<RosArmBridge> m_arm_bridge;
    std::unique_ptr<RosTrailerBridge> m_trailer_bridge;
#endif
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_rocks;
    // Rocks that were in the bed when a dump was requested, so the end of the cycle
    // can report whether they actually left it. See ReportDumpOutcome.
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_carried_rocks;
    // Per-wheel sunk latch (tractor wheels first, then trailer). See CheckWheelSinkage.
    std::vector<bool> m_wheel_sunk;
    int m_sink_reports = 0;
    double m_stuck_since = -1.0;
    double m_last_stuck_report = -1.0e9;
    std::vector<double> m_rock_top_heights;

    // Per-trailer-wheel anomaly probe state: last terrain height seen under each
    // wheel, and that wheel's settled vertical tire force as the spike reference.
    double m_height_probe_z = 0.0;
    std::vector<double> m_trailer_wheel_last_height;
    std::vector<double> m_trailer_wheel_static_fz;
    int m_trailer_anomaly_reports = 0;

    // Front-axle normal load at rest (captured after settle), used as the reference
    // for the load-aware traction guard. 0 => guard runs open-loop (assumes full load).
    double m_front_static_load = 0.0;
    chrono::vehicle::DriverInputs m_last_raw_inputs;
    chrono::vehicle::DriverInputs m_last_guarded_inputs;
    mutable long m_guard_steps = 0;
    mutable long m_guard_limited_steps = 0;
};

}  // namespace amd_uw
