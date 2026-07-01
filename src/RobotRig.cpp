#include "RobotRig.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChTypes.h"
#include "chrono/core/ChVector3.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChConstants.h"
#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"

#include "LrvArm.h"
#include "RobotLayout.h"

#ifdef AMD_UW_ENABLE_ROS2
#include "RosArmBridge.h"
#include "RosControllerDriver.h"
#endif

namespace amd_uw {

namespace {

constexpr double rock_collision_activation_radius = 12.0;
constexpr double rock_collision_deactivation_radius = 16.0;

double VecNorm(const chrono::ChVector3d& v) {
    return std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
}

double PlanarDistance2(const chrono::ChVector3d& a, const chrono::ChVector3d& b) {
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return dx * dx + dy * dy;
}

}  // namespace

class DriverWrapper : public chrono::vehicle::ChDriver {
  public:
    explicit DriverWrapper(chrono::vehicle::ChVehicle& vehicle) : ChDriver(vehicle), m_hold_brake(true) {
        m_throttle = 0.0;
        m_steering = 0.0;
        m_braking = 1.0;
    }

    void Set(std::shared_ptr<chrono::vehicle::ChInteractiveDriver> driver) { m_driver = driver; }

    void Synchronize(double time) override {
        if (!m_driver)
            return;

        m_driver->Synchronize(time);
        m_throttle = m_driver->GetThrottle();
        m_steering = m_driver->GetSteering();

        if (m_hold_brake && (m_throttle > 1e-3 || std::abs(m_steering) > 1e-3)) {
            m_hold_brake = false;
        }

        m_braking = m_hold_brake ? 1.0 : m_driver->GetBraking();
    }

    void Advance(double step) override {
        if (m_driver)
            m_driver->Advance(step);
    }

  private:
    std::shared_ptr<chrono::vehicle::ChInteractiveDriver> m_driver;
    bool m_hold_brake;
};

RobotRig::RobotRig(chrono::ChContactMethod contact_method,
                   int rank,
                   int robot_index,
                   int num_robots,
                   double tire_step_size,
                   double render_step_size)
    : m_rank(rank),
      m_robot_index(robot_index),
      m_num_robots(num_robots),
      m_contact_method(contact_method),
      m_tire_step_size(tire_step_size),
      m_render_step_size(render_step_size),
      m_vehicle(std::make_unique<chrono::vehicle::WheeledVehicle>(
          chrono::vehicle::GetVehicleDataFile("LRV/Polaris.json"), contact_method)) {
    GetSystem()->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    GetSystem()->SetSleepingAllowed(true);
}

RobotRig::~RobotRig() = default;

chrono::ChSystem* RobotRig::GetSystem() const {
    return m_vehicle->GetSystem();
}

chrono::vehicle::WheeledVehicle* RobotRig::GetVehicle() const {
    return m_vehicle.get();
}

std::shared_ptr<chrono::vehicle::WheeledTrailer> RobotRig::GetTrailer() const {
    return m_trailer;
}

chrono::vehicle::ChDriver* RobotRig::GetDriver() const {
    return m_driver.get();
}

const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& RobotRig::GetRocks() const {
    return m_rocks;
}

void RobotRig::InitializeOnTerrain(chrono::vehicle::RigidTerrain& terrain,
                                   const std::shared_ptr<chrono::ChContactMaterial>& rock_mat,
                                   const std::string& chrono_data_path,
                                   const std::string& amd_uw_data_path,
                                   double start_spacing,
                                   double height_probe_z,
                                   double vehicle_start_clearance,
                                   double seat_clearance,
                                   double settle_time,
                                   double step_size,
                                   const RockFieldConfig& rock_field_config) {
    m_rock_top_heights.clear();
    m_rocks = AddRockFields(GetSystem(), terrain, rock_mat, chrono_data_path, amd_uw_data_path, m_robot_index,
                            m_num_robots, start_spacing, height_probe_z, rock_field_config, &m_rock_top_heights);

    const chrono::ChVector3d start_ground = InitialGroundPositionForRobot(m_robot_index, m_num_robots, start_spacing);
    const double start_x = start_ground.x();
    const double start_y = start_ground.y();
    const double start_z = terrain.GetHeight(chrono::ChVector3d(start_x, start_y, height_probe_z)) +
                           vehicle_start_clearance;
    const chrono::ChVector3d init_loc(start_x, start_y, start_z);
    const double init_heading_deg = InitialHeadingDegForRobot(m_robot_index);
    const chrono::ChQuaternion<> init_rot = chrono::QuatFromAngleZ(init_heading_deg * chrono::CH_DEG_TO_RAD);

    std::vector<chrono::ChBody*> preexisting_bodies;
    for (const auto& body : GetSystem()->GetBodies())
        preexisting_bodies.push_back(body.get());

    InitializeVehicle(chrono::ChCoordsys<>(init_loc, init_rot));
    InitializeTrailer();
    ReseatRig(terrain, preexisting_bodies, height_probe_z, seat_clearance);
    InitializeArm(amd_uw_data_path);
    InitializeTrailerBed();
    for (const auto& body : GetSystem()->GetBodies())
        body->SetSleepingAllowed(false);
    for (const auto& rock : m_rocks)
        rock->SetSleepingAllowed(true);
    InitializeDriver();
#ifdef AMD_UW_ENABLE_ROS2
    InitializeArmBridge(height_probe_z);
#endif
    Settle(terrain, settle_time, step_size);
    if (settle_time > 0) {
        for (const auto& rock : m_rocks)
            rock->SetSleeping(true);
    }
    UpdateRockCollisionActivation();

    chrono::synchrono::SynLog() << "Rank " << m_rank << " owns robot index " << m_robot_index << " and "
                                << m_rocks.size() << " dynamic rocks.\n";
}

void RobotRig::InitializeVehicle(const chrono::ChCoordsys<>& init_pos) {
    m_vehicle->Initialize(init_pos);
    m_vehicle->GetChassis()->SetFixed(false);
    m_vehicle->SetChassisVisualizationType(chrono::VisualizationType::MESH);
    m_vehicle->SetSuspensionVisualizationType(chrono::VisualizationType::PRIMITIVES);
    m_vehicle->SetSteeringVisualizationType(chrono::VisualizationType::PRIMITIVES);
    m_vehicle->SetWheelVisualizationType(chrono::VisualizationType::MESH);

    auto engine = chrono::vehicle::ReadEngineJSON(
        chrono::vehicle::GetVehicleDataFile("LRV/Polaris_EngineSimpleMap.json"));
    auto transmission = chrono::vehicle::ReadTransmissionJSON(
        chrono::vehicle::GetVehicleDataFile("LRV/Polaris_AutomaticTransmissionSimpleMap.json"));
    auto powertrain = chrono_types::make_shared<chrono::vehicle::ChPowertrainAssembly>(engine, transmission);
    m_vehicle->InitializePowertrain(powertrain);

    for (auto& axle : m_vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = chrono::vehicle::ReadTireJSON(
                chrono::vehicle::GetVehicleDataFile("LRV/Polaris_RigidTire.json"));
            m_vehicle->InitializeTire(tire, wheel, chrono::VisualizationType::MESH);
            tire->SetStepsize(m_tire_step_size);
        }
    }
}

void RobotRig::InitializeTrailer() {
    m_trailer = chrono_types::make_shared<chrono::vehicle::WheeledTrailer>(
        GetSystem(), chrono::vehicle::GetVehicleDataFile("LRV_Wagon/Polaris.json"));
    m_trailer->Initialize(m_vehicle->GetChassis());
    m_trailer->SetChassisVisualizationType(chrono::VisualizationType::PRIMITIVES);
    m_trailer->SetSuspensionVisualizationType(chrono::VisualizationType::PRIMITIVES);
    m_trailer->SetWheelVisualizationType(chrono::VisualizationType::PRIMITIVES);

    for (auto& axle : m_trailer->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = chrono::vehicle::ReadTireJSON(
                chrono::vehicle::GetVehicleDataFile("LRV/Polaris_RigidTire.json"));
            m_trailer->InitializeTire(tire, wheel, chrono::VisualizationType::MESH);
            tire->SetStepsize(m_tire_step_size);
        }
    }
}

void RobotRig::InitializeArm(const std::string& amd_uw_data_path) {
    const chrono::ChVector3d mount_offset(-1.1, 0.0, 0.1);
    const auto chassis = m_vehicle->GetChassisBody();
    const chrono::ChVector3d mount_pos = chassis->GetPos() + chassis->GetRot().Rotate(mount_offset);
    m_arm = std::make_unique<LrvArm>(GetSystem(), chassis, amd_uw_data_path, mount_pos, chassis->GetRot());
}

void RobotRig::ReseatRig(chrono::vehicle::RigidTerrain& terrain,
                         const std::vector<chrono::ChBody*>& preexisting_bodies,
                         double height_probe_z,
                         double seat_clearance) {
    std::set<chrono::ChBody*> preexisting(preexisting_bodies.begin(), preexisting_bodies.end());
    double min_clearance = std::numeric_limits<double>::infinity();
    auto consider_wheel = [&](const auto& wheel) {
        if (!wheel)
            return;
        const auto& tire = wheel->GetTire();
        const double radius = tire ? tire->GetRadius() : 0.0;
        const chrono::ChVector3d p = wheel->GetPos();
        const double bottom = p.z() - radius;
        const double terrain_under_wheel = terrain.GetHeight(chrono::ChVector3d(p.x(), p.y(), height_probe_z));
        min_clearance = std::min(min_clearance, bottom - terrain_under_wheel);
    };
    for (auto& axle : m_vehicle->GetAxles())
        for (auto& wheel : axle->GetWheels())
            consider_wheel(wheel);
    for (auto& axle : m_trailer->GetAxles())
        for (auto& wheel : axle->GetWheels())
            consider_wheel(wheel);

    const double drop = min_clearance - seat_clearance;

    for (const auto& body : GetSystem()->GetBodies()) {
        if (preexisting.count(body.get()))
            continue;
        const chrono::ChVector3d p = body->GetPos();
        body->SetPos(chrono::ChVector3d(p.x(), p.y(), p.z() - drop));
    }

    chrono::synchrono::SynLog() << "Re-seated rank " << m_rank << " rig: lowered by " << drop << " m.\n";
}

void RobotRig::InitializeTrailerBed() {
    auto bed_mat = chrono::ChContactMaterial::DefaultMaterial(m_contact_method);
    bed_mat->SetFriction(0.9f);

    const auto chassis = m_trailer->GetChassis()->GetBody();
    const chrono::ChVector3d offset(0.0, 0.0, 0.03);  // bed floor just above the trailer chassis
    const chrono::ChVector3d bed_pos = chassis->GetPos() + chassis->GetRot().Rotate(offset);

    // Open dumping tub (floor + front +x and both +/-y walls, open rear -x), a
    // DYNAMIC body carried by the trailer through a lateral-axis revolute motor
    // held flat. Unlike the old fixed/teleported plate, a jointed dynamic bed has
    // real velocity, so friction keeps placed rocks aboard while driving. Mirrors
    // the Python TrailerDumpBed; the motor can later run a dump cycle.
    const double ex = 1.0, ey = 1.2;   // footprint: x along the trailer, y across
    const double wall_h = 0.15, t = 0.03;

    m_trailer_bed = chrono_types::make_shared<chrono::ChBody>();
    m_trailer_bed->SetName("trailer_bed");
    m_trailer_bed->SetPos(bed_pos);
    m_trailer_bed->SetRot(chassis->GetRot());
    const double mass = 30.0;
    m_trailer_bed->SetMass(mass);
    m_trailer_bed->SetInertiaXX(chrono::ChVector3d(mass / 12.0 * (ey * ey + wall_h * wall_h),
                                                   mass / 12.0 * (ex * ex + wall_h * wall_h),
                                                   mass / 12.0 * (ex * ex + ey * ey)));

    auto add_box = [&](double sx, double sy, double sz, double cx, double cy, double cz) {
        m_trailer_bed->AddCollisionShape(
            chrono_types::make_shared<chrono::ChCollisionShapeBox>(bed_mat, sx, sy, sz),
            chrono::ChFramed(chrono::ChVector3d(cx, cy, cz), chrono::QUNIT));
    };
    add_box(ex, ey, t, 0.0, 0.0, 0.0);                       // floor
    add_box(t, ey, wall_h, ex / 2.0, 0.0, wall_h / 2.0);     // +x front wall
    add_box(ex, t, wall_h, 0.0, ey / 2.0, wall_h / 2.0);     // +y left wall
    add_box(ex, t, wall_h, 0.0, -ey / 2.0, wall_h / 2.0);    // -y right wall
    m_trailer_bed->EnableCollision(true);
    GetSystem()->AddBody(m_trailer_bed);

    // Revolute motor about the chassis lateral (Y) axis. A rotation motor turns
    // about its frame Z, so rotate the frame +90 deg about X to land Z on the
    // chassis Y axis. Held at 0 => bed stays flat (rigidly carried).
    const chrono::ChQuaternion<> frame_rot = chassis->GetRot() * chrono::QuatFromAngleX(chrono::CH_PI_2);
    m_trailer_bed_motor = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
    m_trailer_bed_motor->Initialize(m_trailer_bed, chassis, chrono::ChFramed(bed_pos, frame_rot));
    m_trailer_bed_motor->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.0));
    GetSystem()->AddLink(m_trailer_bed_motor);
}

void RobotRig::DumpTrailerBed() {
    // Placeholder for a future dump cycle: ramp the motor angle up to tip the bed
    // and slide the load off the open rear, then return to flat. Not wired to ROS.
    if (m_trailer_bed_motor)
        m_trailer_bed_motor->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.0));
}

void RobotRig::InitializeDriver() {
#ifdef AMD_UW_ENABLE_ROS2
    m_driver = std::make_unique<RosControllerDriver>(*m_vehicle, m_rank, m_rocks);
#else
    auto interactive_driver = std::make_unique<DriverWrapper>(*m_vehicle);
    m_irr_driver = chrono_types::make_shared<chrono::vehicle::ChInteractiveDriver>(*m_vehicle);
    m_irr_driver->SetSteeringDelta(m_render_step_size / 1.0);
    m_irr_driver->SetThrottleDelta(m_render_step_size / 1.0);
    m_irr_driver->SetBrakingDelta(m_render_step_size / 0.3);
    m_irr_driver->Initialize();
    interactive_driver->Set(m_irr_driver);
    m_driver = std::move(interactive_driver);
#endif
}

#ifdef AMD_UW_ENABLE_ROS2
void RobotRig::InitializeArmBridge(double height_probe_z) {
    m_arm_bridge =
        std::make_unique<RosArmBridge>(m_rank, *m_arm, m_rocks, m_rock_top_heights, m_trailer, height_probe_z);
}
#endif

void RobotRig::Settle(chrono::vehicle::RigidTerrain& terrain, double settle_time, double step_size) {
    if (settle_time <= 0)
        return;

    chrono::vehicle::DriverInputs brake_inputs = {0.0, 0.0, 1.0, 0.0};
    const int settle_steps = static_cast<int>(std::ceil(settle_time / step_size));

    for (int i = 0; i < settle_steps; i++) {
        const double time = GetSystem()->GetChTime();
        terrain.Synchronize(time);
        m_vehicle->Synchronize(time, brake_inputs, terrain);
        m_trailer->Synchronize(time, brake_inputs, terrain);
        terrain.Advance(step_size);
        m_vehicle->Advance(step_size);
        m_trailer->Advance(step_size);
    }

    for (const auto& body : GetSystem()->GetBodies()) {
        body->SetPosDt(chrono::VNULL);
        body->SetAngVelLocal(chrono::VNULL);
        body->SetPosDt2(chrono::VNULL);
    }

    GetSystem()->SetChTime(0.0);
}

void RobotRig::Synchronize(double time, chrono::vehicle::RigidTerrain& terrain) {
    UpdateRockCollisionActivation();
    m_driver->Synchronize(time);
#ifdef AMD_UW_ENABLE_ROS2
    if (m_arm_bridge)
        m_arm_bridge->Synchronize(time, terrain);
#else
    if (m_arm)
        m_arm->Update(time);
#endif
    const auto driver_inputs = m_driver->GetInputs();
    m_vehicle->Synchronize(time, driver_inputs, terrain);
    m_trailer->Synchronize(time, driver_inputs, terrain);
}

void RobotRig::Advance(double step) {
    m_driver->Advance(step);
    m_vehicle->Advance(step);
    m_trailer->Advance(step);
}

chrono::vehicle::DriverInputs RobotRig::GetDriverInputs() const {
    return m_driver->GetInputs();
}

void RobotRig::UpdateRockCollisionActivation() {
    if (m_rocks.empty())
        return;

    const chrono::ChVector3d vehicle_pos = m_vehicle->GetChassisBody()->GetPos();
    chrono::ChVector3d trailer_pos = vehicle_pos;
    if (m_trailer && m_trailer->GetChassis())
        trailer_pos = m_trailer->GetChassis()->GetPos();

    const double activate2 = rock_collision_activation_radius * rock_collision_activation_radius;
    const double deactivate2 = rock_collision_deactivation_radius * rock_collision_deactivation_radius;

    // The arm freezes its target rock (fixed + collision off) while grabbing;
    // don't fight that here.
    const auto active_rock = m_arm ? m_arm->GetActiveRock() : nullptr;

    for (const auto& rock : m_rocks) {
        if (rock == active_rock)
            continue;
#ifdef AMD_UW_ENABLE_ROS2
        // Rocks welded to the trailer bed have their collision managed by the
        // arm bridge; leave them alone.
        if (m_arm_bridge) {
            const auto& welded = m_arm_bridge->WeldedRocks();
            if (std::find(welded.begin(), welded.end(), rock) != welded.end())
                continue;
        }
#endif
        const auto rock_pos = rock->GetPos();
        const double dist2 = std::min(PlanarDistance2(rock_pos, vehicle_pos), PlanarDistance2(rock_pos, trailer_pos));
        if (!rock->IsCollisionEnabled() && dist2 <= activate2) {
            rock->EnableCollision(true);
            rock->SetSleeping(false);
        } else if (rock->IsCollisionEnabled() && rock->IsSleeping() && dist2 >= deactivate2) {
            rock->EnableCollision(false);
        }
    }
}

void RobotRig::LogMotionIfNeeded(int step_number,
                                 int motion_log_steps,
                                 chrono::vehicle::RigidTerrain& terrain) const {
    if (motion_log_steps <= 0 || step_number % motion_log_steps != 0)
        return;

    const auto chassis = m_vehicle->GetChassisBody();
    const chrono::ChVector3d p = chassis->GetPos();
    const chrono::ChVector3d v = chassis->GetPosDt();
    const chrono::ChVector3d a = chassis->GetPosDt2();
    const chrono::ChVector3d w = chassis->GetAngVelParent();
    const chrono::ChVector3d chassis_contact = chassis->GetContactForce();
    double tire_force_sum = 0.0;
    double tire_force_z = 0.0;

    for (const auto& axle : m_vehicle->GetAxles()) {
        for (const auto& wheel : axle->GetWheels()) {
            const auto& tire = wheel->GetTire();
            if (!tire)
                continue;
            const auto force = tire->ReportTireForce(&terrain).force;
            tire_force_sum += VecNorm(force);
            tire_force_z += force.z();
        }
    }

    chrono::synchrono::SynLog() << "motion rank=" << m_rank << " t=" << GetSystem()->GetChTime() << " pos=("
                                << p.x() << "," << p.y() << "," << p.z() << ") speed=" << VecNorm(v)
                                << " accel=" << VecNorm(a) << " ang_speed=" << VecNorm(w)
                                << " chassis_contact=" << VecNorm(chassis_contact)
                                << " tire_force_sum=" << tire_force_sum << " tire_force_z=" << tire_force_z << "\n";
}

}  // namespace amd_uw
