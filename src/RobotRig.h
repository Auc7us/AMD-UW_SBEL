#pragma once

#include <map>
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
    // Dump tub and hinged tailgate, exposed so SynTrailerBedAgent can broadcast their
    // poses -- no stock agent carries them.
    std::shared_ptr<chrono::ChBody> GetTrailerBed() const { return m_trailer_bed; }
    std::shared_ptr<chrono::ChBody> GetTrailerTailgate() const { return m_trailer_tailgate; }
    chrono::vehicle::ChDriver* GetDriver() const;
    // The rover's manipulator, exposed so its bodies can be broadcast to the sensor
    // rank like the builder's.
    LrvArm* GetArm() const { return m_arm.get(); }
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
    // Current harvest cycle. The collector's lane angle is
    // HarvestLaneAngleRad(robot_index, num_robots, GetHarvestCycle()).
    int GetHarvestCycle() const { return m_harvest_cycle; }

    // Rocks this rank's collector has delivered: dumped out of the bed, come to rest,
    // and frozen. These are the builder's feedstock once its seed heap is gone -- they
    // stay fixed exactly as the seed heap's rocks do, so the pile does not creep, and
    // LrvArm unfixes whichever one its gripper locks on to.
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> GetDeliveredRocks() const;
    // True once this rank's physics has produced a non-finite value. Nothing this
    // rig reports afterwards is meaningful, so the caller should stop the run
    // rather than spend hours simulating a dead rank.
    bool HasDiverged() const { return m_diverged; }

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
    // Freeze rocks that have been dumped and come to rest, so they stop creeping away
    // from the pile. They stay frozen: see the comment above GetDeliveredRocks.
    void UpdateDumpedRockFreeze(double time);
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
    // Lock the axle differentials for straight running, release them to turn.
    void UpdateAxleDifferentialLock(double steering);
    // Engaged at initialization, so this starts true.
    bool m_axle_diff_locked = true;
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
    // Hard end-stop on every upright's rotation about its kingpin. See the definition:
    // the tie rod is a distance constraint with TWO solutions, and the second one is a
    // knuckle turned ~108 deg, so the stop exists to make that root unreachable.
    void ApplySteeringStops(double time);

    // One entry per upright, built on the first ApplySteeringStops call.
    struct SteeringStop {
        std::shared_ptr<chrono::ChBody> upright;
        chrono::ChQuaternion<> rest;      // upright rotation in the chassis frame at t=0
        unsigned int accumulator = 0;
        bool engaged = false;            // latched for reporting, not for physics
        bool alarmed = false;            // ditto; the alarm fires every step otherwise
        const char* label = "";
    };
    std::vector<SteeringStop> m_steering_stops;
    // One accumulator on the chassis carries the summed reaction of all four stops, so
    // the barrier is an internal torque pair and not a torque out of nowhere.
    unsigned int m_steering_stop_reaction = 0;
    bool m_steering_stops_ready = false;
    int m_steering_stop_reports = 0;
    // Names the body that is blowing up while it is still finite, and the
    // constraints pulling on it. See the definition for why NaN itself is too
    // late to diagnose from.
    void CheckDivergence(double time);
    // Rotate this rank's lane one step and spawn its next set of rocks. Called on the
    // dump-complete edge; see the definition for why it must rebind collision.
    void StartNextHarvestCycle(double time);
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
    // Dumped rocks waiting to come to rest. See UpdateDumpedRockFreeze.
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_settling_rocks;
    // Delivered feedstock: dumped, settled, frozen, and offered to the builder. Grows
    // by one load per harvest cycle and is never pruned -- the builder tracks which of
    // them it has already consumed.
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_delivered_rocks;
    std::map<const chrono::ChBody*, double> m_rock_still_since;
    std::vector<bool> m_wheel_sunk;
    int m_sink_reports = 0;
    double m_stuck_since = -1.0;
    double m_last_stuck_report = -1.0e9;
    // Divergence tripwire state. m_diverged latches on the first non-finite body:
    // once true this rank's physics is dead and no other reading from it means
    // anything.
    double m_last_divergence_scan = -1.0e9;
    int m_divergence_reports = 0;
    bool m_diverged = false;
    std::vector<double> m_rock_top_heights;

    // Per-trailer-wheel anomaly probe state: last terrain height seen under each
    // wheel, and that wheel's settled vertical tire force as the spike reference.
    double m_height_probe_z = 0.0;

    // Retained from InitializeOnTerrain so a later harvest cycle can spawn rocks
    // exactly as the first one did. The terrain outlives the rig (main owns it).
    chrono::vehicle::ChTerrain* m_terrain = nullptr;
    std::shared_ptr<chrono::ChContactMaterial> m_rock_mat;
    std::string m_chrono_data_path;
    std::string m_amd_uw_data_path;
    RockFieldConfig m_rock_field_config;
    // Harvest cycle index. 0 is the initial lane; each completed dump advances it and
    // rotates the lane cycle_rotation_rad counter-clockwise about the site centre.
    int m_harvest_cycle = 0;
    std::vector<double> m_trailer_wheel_last_height;
    std::vector<double> m_trailer_wheel_static_fz;
    int m_trailer_anomaly_reports = 0;

    // Front-axle normal load at rest (captured after settle), used as the reference
    // for the load-aware traction guard. 0 => guard runs open-loop (assumes full load).
    double m_front_static_load = 0.0;
    // Initialised for the same reason as BuilderRig::m_driver_inputs: DriverInputs has
    // no default member initialisers. CheckStuck reads m_last_guarded_inputs.m_throttle,
    // so leaving it indeterminate let the stuck detector act on garbage before the first
    // controller message.
    chrono::vehicle::DriverInputs m_last_raw_inputs{0.0, 0.0, 1.0, 0.0};
    chrono::vehicle::DriverInputs m_last_guarded_inputs{0.0, 0.0, 1.0, 0.0};
    mutable long m_guard_steps = 0;
    mutable long m_guard_limited_steps = 0;
};

}  // namespace amd_uw
