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
    std::shared_ptr<chrono::vehicle::WheeledTrailer> GetTrailer() const;
    chrono::vehicle::ChDriver* GetDriver() const;
    const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& GetRocks() const;

    void InitializeOnTerrain(chrono::vehicle::ChTerrain& terrain,
                             const std::shared_ptr<chrono::ChContactMaterial>& rock_mat,
                             const std::string& chrono_data_path,
                             const std::string& amd_uw_data_path,
                             double start_spacing,
                             double height_probe_z,
                             double vehicle_start_clearance,
                             double seat_clearance,
                             double settle_time,
                             double step_size,
                             const RockFieldConfig& rock_field_config);

    void Synchronize(double time, chrono::vehicle::ChTerrain& terrain);
    void Advance(double step);
    chrono::vehicle::DriverInputs GetDriverInputs() const;
    void LogMotionIfNeeded(int step_number,
                           int motion_log_steps,
                           chrono::vehicle::ChTerrain& terrain) const;

  private:
    void InitializeVehicle(const chrono::ChCoordsys<>& init_pos);
    void InitializeTrailer();
    // Add radial box "grousers" around a wheel spindle, on top of the RigidTire
    // cylinder (one compound/union collision). Primitives only -> cheap and stable;
    // the bricks imprint tread marks / catch soil once the SCM grid is fine enough.
    void AddGrouserBricks(const std::shared_ptr<chrono::ChBody>& spindle);
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
    std::unique_ptr<chrono::vehicle::ChDriver> m_driver;
    std::shared_ptr<chrono::vehicle::ChInteractiveDriver> m_irr_driver;
    std::unique_ptr<LrvArm> m_arm;
#ifdef AMD_UW_ENABLE_ROS2
    std::unique_ptr<RosArmBridge> m_arm_bridge;
#endif
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_rocks;
    std::vector<double> m_rock_top_heights;

    // Front-axle normal load at rest (captured after settle), used as the reference
    // for the load-aware traction guard. 0 => guard runs open-loop (assumes full load).
    double m_front_static_load = 0.0;
};

}  // namespace amd_uw
