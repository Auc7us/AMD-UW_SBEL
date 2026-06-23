#pragma once

#include <memory>
#include <string>
#include <vector>

#include "RockField.h"

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

namespace amd_uw {

class DriverWrapper;

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
    chrono::vehicle::ChInteractiveDriver* GetInteractiveDriver() const;
    const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& GetRocks() const;

    void InitializeOnTerrain(chrono::vehicle::RigidTerrain& terrain,
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

    void Synchronize(double time, chrono::vehicle::RigidTerrain& terrain);
    void Advance(double step);
    chrono::vehicle::DriverInputs GetDriverInputs() const;
    void LogMotionIfNeeded(int step_number,
                           int motion_log_steps,
                           chrono::vehicle::RigidTerrain& terrain) const;

  private:
    void InitializeVehicle(const chrono::ChCoordsys<>& init_pos);
    void InitializeTrailer();
    void ReseatRig(chrono::vehicle::RigidTerrain& terrain,
                   const std::vector<chrono::ChBody*>& preexisting_bodies,
                   double height_probe_z,
                   double seat_clearance);
    void InitializeTrailerBed();
    void InitializeDriver();
    void Settle(chrono::vehicle::RigidTerrain& terrain, double settle_time, double step_size);
    void UpdateAttachments();

    int m_rank;
    int m_robot_index;
    int m_num_robots;
    chrono::ChContactMethod m_contact_method;
    double m_tire_step_size;
    double m_render_step_size;

    std::unique_ptr<chrono::vehicle::WheeledVehicle> m_vehicle;
    std::shared_ptr<chrono::vehicle::WheeledTrailer> m_trailer;
    std::shared_ptr<chrono::ChBodyEasyBox> m_trailer_bed;
    std::unique_ptr<DriverWrapper> m_driver;
    std::shared_ptr<chrono::vehicle::ChInteractiveDriver> m_irr_driver;
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_rocks;
};

}  // namespace amd_uw
