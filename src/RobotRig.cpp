#include "RobotRig.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "chrono/assets/ChVisualShapeBox.h"
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
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"

#include "LrvArm.h"
#include "MaterialUtils.h"
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

void RobotRig::InitializeOnTerrain(chrono::vehicle::ChTerrain& terrain,
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
    // Create the trailer bed BEFORE the arm. InitializeArm runs the SolidWorks
    // importer (ImportSolidWorksSystem, embedded Python), which disrupts the
    // collision system so that bodies added AFTER it never register contacts:
    // in the importer build rocks fell straight through the bed, while the
    // non-importer build caught them on the identical bed. Bodies created before
    // the import (the rocks, and now the bed) bind normally and collide.
    InitializeTrailerBed();
    InitializeArm(amd_uw_data_path);
    for (const auto& body : GetSystem()->GetBodies())
        body->SetSleepingAllowed(false);
    for (const auto& rock : m_rocks)
        rock->SetSleepingAllowed(true);
    InitializeDriver();
#ifdef AMD_UW_ENABLE_ROS2
    InitializeArmBridge(height_probe_z);
#endif
    // If this is deformable SCM terrain, restrict soil computation to moving patches
    // under each wheel BEFORE settling. Otherwise SCM makes the entire ~450k-node grid
    // active and every settle step recomputes the whole grid -- which stalls startup
    // for minutes. Must run before Settle() (which steps the sim 2000x).
    if (auto* scm = dynamic_cast<chrono::vehicle::SCMTerrain*>(&terrain)) {
        const chrono::ChVector3d wheel_domain(1.0, 0.7, 1.0);
        for (auto& axle : m_vehicle->GetAxles())
            for (auto& wheel : axle->GetWheels())
                scm->AddActiveDomain(wheel->GetSpindle(), chrono::VNULL, wheel_domain);
        for (auto& axle : m_trailer->GetAxles())
            for (auto& wheel : axle->GetWheels())
                scm->AddActiveDomain(wheel->GetSpindle(), chrono::VNULL, wheel_domain);
        // A small active domain per rock so SCM actually supports them. Without it,
        // rocks lie outside every active domain, get no soil reaction, and fall
        // through the terrain during settle (spawning "under" the SCM surface).
        const chrono::ChVector3d rock_domain(1.0, 1.0, 1.0);
        for (auto& rock : m_rocks)
            scm->AddActiveDomain(rock, chrono::ChVector3d(0.0, 0.0, 0.3), rock_domain);
    }
    Settle(terrain, settle_time, step_size);
    if (settle_time > 0) {
        for (const auto& rock : m_rocks)
            rock->SetSleeping(true);
    }
    // Reference front-axle load at rest for the load-aware traction guard.
    m_front_static_load = FrontAxleNormalLoad(terrain);
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

    // Lunar-scaled engine torque (~1/5 of the Earth-tuned map). At 1.62 m/s^2 the
    // rover weighs ~6x less, so the original ~397 Nm peak wheelie'd the front off
    // the ground and killed steering grip. Tune the SCALE in the generator or edit
    // Polaris_EngineSimpleMap_lunar.json directly.
    auto engine = chrono::vehicle::ReadEngineJSON(
        chrono::vehicle::GetVehicleDataFile("LRV/Polaris_EngineSimpleMap_lunar.json"));
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
            AddGrouserBricks(wheel->GetSpindle());
        }
    }

    // Front ballast: a mass rigidly welded to the chassis over the front axle to
    // keep the front tires loaded so they don't lift/bounce under acceleration and
    // keep steering grip. Tune front_ballast_mass (and the height) to taste; set to
    // 0 to disable.
    const double front_ballast_mass = 250.0;
    if (front_ballast_mass > 0.0) {
        const auto chassis = m_vehicle->GetChassis()->GetBody();
        const auto front_spindle = m_vehicle->GetAxles().front()->GetWheels().front()->GetSpindle();
        // Place over the front axle (chassis-local x from the front spindle), at the
        // chassis reference height (low, so it also drops the CG a little).
        const chrono::ChVector3d front_local = chassis->TransformPointParentToLocal(front_spindle->GetPos());
        const chrono::ChVector3d ballast_local(front_local.x(), 0.0, 0.0);
        const chrono::ChVector3d ballast_world = chassis->TransformPointLocalToParent(ballast_local);

        auto ballast = chrono_types::make_shared<chrono::ChBody>();
        ballast->SetName("front_ballast");
        ballast->SetPos(ballast_world);
        ballast->SetRot(chassis->GetRot());
        ballast->SetMass(front_ballast_mass);
        const double r = 0.2;  // inertia of a compact lump
        ballast->SetInertiaXX(chrono::ChVector3d(0.4 * front_ballast_mass * r * r,
                                                 0.4 * front_ballast_mass * r * r,
                                                 0.4 * front_ballast_mass * r * r));
        GetSystem()->AddBody(ballast);

        auto weld = chrono_types::make_shared<chrono::ChLinkLockLock>();
        weld->SetName("front_ballast_weld");
        weld->Initialize(ballast, chassis, chrono::ChFramed(ballast_world, chassis->GetRot()));
        GetSystem()->AddLink(weld);
    }
}

void RobotRig::InitializeTrailer() {
    m_trailer = chrono_types::make_shared<chrono::vehicle::WheeledTrailer>(
        GetSystem(), chrono::vehicle::GetVehicleDataFile("LRV_Wagon/Polaris.json"));
    m_trailer->Initialize(m_vehicle->GetChassis());
    m_trailer->SetChassisVisualizationType(chrono::VisualizationType::NONE);
    m_trailer->SetSuspensionVisualizationType(chrono::VisualizationType::PRIMITIVES);
    m_trailer->SetWheelVisualizationType(chrono::VisualizationType::PRIMITIVES);

    for (auto& axle : m_trailer->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = chrono::vehicle::ReadTireJSON(
                chrono::vehicle::GetVehicleDataFile("LRV/Polaris_RigidTire.json"));
            m_trailer->InitializeTire(tire, wheel, chrono::VisualizationType::MESH);
            tire->SetStepsize(m_tire_step_size);
            AddGrouserBricks(wheel->GetSpindle());
        }
    }
}

void RobotRig::AddGrouserBricks(const std::shared_ptr<chrono::ChBody>& spindle) {
    // Approximate grousers as radial box "bricks" spaced around the tire. They attach
    // to the same spindle as the RigidTire cylinder, so the wheel's collision is the
    // union of cylinder + bricks -- all primitives, so contact stays cheap and stable.
    // The bricks imprint tread marks / catch soil once the SCM grid resolves them.
    constexpr int n_grousers = 10;
    constexpr double carcass_r = 0.4089;  // = RigidTire cylinder Radius (carcass)
    constexpr double height = 0.02;       // radial protrusion (grouser tips ~0.429)
    constexpr double axial = 0.26;        // length along the axle (< tire width)
    constexpr double tangential = 0.05;   // thickness around the circumference
    const auto mat = MakeContactMaterial(m_contact_method, 0.9f);
    for (int i = 0; i < n_grousers; ++i) {
        // Spindle spins about its local Y; the wheel lies in the local X-Z plane. Place
        // each brick at radius carcass_r+height/2 in direction (cos,0,sin) and rotate
        // about Y so the box's local X points radially outward.
        const double th = i * 2.0 * chrono::CH_PI / n_grousers;
        const chrono::ChVector3d pos(std::cos(th) * (carcass_r + height / 2.0), 0.0,
                                     std::sin(th) * (carcass_r + height / 2.0));
        spindle->AddCollisionShape(
            chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat, height, axial, tangential),
            chrono::ChFramed(pos, chrono::QuatFromAngleY(-th)));
    }
}

void RobotRig::InitializeArm(const std::string& amd_uw_data_path) {
    const chrono::ChVector3d mount_offset(-1.1, 0.0, 0.1);
    const auto chassis = m_vehicle->GetChassisBody();
    const chrono::ChVector3d mount_pos = chassis->GetPos() + chassis->GetRot().Rotate(mount_offset);
    m_arm = std::make_unique<LrvArm>(GetSystem(), chassis, amd_uw_data_path, mount_pos, chassis->GetRot());
}

void RobotRig::ReseatRig(chrono::vehicle::ChTerrain& terrain,
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
    auto bed_mat = MakeContactMaterial(m_contact_method, 0.9f);

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

    const chrono::ChColor bed_color(0.75f, 0.5f, 0.5f);

    auto add_box = [&](const std::shared_ptr<chrono::ChBody>& body,
                       double sx,
                       double sy,
                       double sz,
                       double cx,
                       double cy,
                       double cz) {
        const chrono::ChFramed frame(chrono::ChVector3d(cx, cy, cz), chrono::QUNIT);

        body->AddCollisionShape(
            chrono_types::make_shared<chrono::ChCollisionShapeBox>(bed_mat, sx, sy, sz),
            frame);

        auto visual = chrono_types::make_shared<chrono::ChVisualShapeBox>(sx, sy, sz);
        visual->SetColor(bed_color);
        body->AddVisualShape(visual, frame);
    };
    add_box(m_trailer_bed, ex, ey, t, 0.0, 0.0, 0.0);                       // floor
    add_box(m_trailer_bed, t, ey, wall_h, ex / 2.0, 0.0, wall_h / 2.0);     // +x front wall
    add_box(m_trailer_bed, ex, t, wall_h, 0.0, ey / 2.0, wall_h / 2.0);     // +y left wall
    add_box(m_trailer_bed, ex, t, wall_h, 0.0, -ey / 2.0, wall_h / 2.0);    // -y right wall
    m_trailer_bed->EnableCollision(true);
    GetSystem()->AddBody(m_trailer_bed);

    // Hinged rear tailgate (-x): a separate dynamic body, connected to the bed
    // with a revolute joint about the trailer lateral (Y) axis.
    const chrono::ChVector3d tailgate_center_local(-ex / 2.0 - t / 2.0, 0.0, t / 2.0 + wall_h / 2.0);
    const chrono::ChVector3d tailgate_hinge_local(-ex / 2.0 - t / 2.0, 0.0, t / 2.0);
    const chrono::ChVector3d tailgate_pos = bed_pos + chassis->GetRot().Rotate(tailgate_center_local);

    m_trailer_tailgate = chrono_types::make_shared<chrono::ChBody>();
    m_trailer_tailgate->SetName("trailer_tailgate");
    m_trailer_tailgate->SetPos(tailgate_pos);
    m_trailer_tailgate->SetRot(chassis->GetRot());
    const double tailgate_mass = 5.0;
    m_trailer_tailgate->SetMass(tailgate_mass);
    m_trailer_tailgate->SetInertiaXX(
        chrono::ChVector3d(tailgate_mass / 12.0 * (ey * ey + wall_h * wall_h),
                           tailgate_mass / 12.0 * (t * t + wall_h * wall_h),
                           tailgate_mass / 12.0 * (t * t + ey * ey)));
    add_box(m_trailer_tailgate, t, ey, wall_h, 0.0, 0.0, 0.0);
    m_trailer_tailgate->EnableCollision(true);
    GetSystem()->AddBody(m_trailer_tailgate);

    // Tailgate hinge as a rotation motor held at angle 0 -> the gate stays CLOSED
    // (a free revolute here just dangled open, since a bottom-hinged flap is
    // unstable upright under gravity). Same pattern as the bed motor; drive this
    // angle later to swing the gate open for a dump. Motor turns about frame Z, so
    // rotate +90 deg about X to put Z on the trailer lateral (Y) axis.
    const chrono::ChQuaternion<> tailgate_hinge_rot = chassis->GetRot() * chrono::QuatFromAngleX(chrono::CH_PI_2);
    const chrono::ChVector3d tailgate_hinge_pos = bed_pos + chassis->GetRot().Rotate(tailgate_hinge_local);
    m_trailer_tailgate_hinge = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
    m_trailer_tailgate_hinge->SetName("trailer_tailgate_hinge");
    m_trailer_tailgate_hinge->Initialize(m_trailer_tailgate, m_trailer_bed,
                                         chrono::ChFramed(tailgate_hinge_pos, tailgate_hinge_rot));
    m_trailer_tailgate_hinge->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.0));
    GetSystem()->AddLink(m_trailer_tailgate_hinge);

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

void RobotRig::Settle(chrono::vehicle::ChTerrain& terrain, double settle_time, double step_size) {
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

double RobotRig::FrontAxleNormalLoad(chrono::vehicle::ChTerrain& terrain) const {
    const auto& axles = m_vehicle->GetAxles();
    if (axles.empty())
        return 0.0;
    double fz = 0.0;
    for (auto& wheel : axles.front()->GetWheels()) {
        const auto& tire = wheel->GetTire();
        if (tire)
            fz += std::abs(tire->ReportTireForce(&terrain).force.z());
    }
    return fz;
}

void RobotRig::ApplyTractionGuard(chrono::vehicle::DriverInputs& in, chrono::vehicle::ChTerrain& terrain) const {
    // --- Tunables ---
    constexpr double mu = 0.8;          // tire-terrain friction used for the limit
    constexpr double wheelbase = 2.5;   // m
    constexpr double max_steer = 0.6;   // road-wheel angle (rad) at |steering|=1 (matches controller)
    constexpr double brake_gain = 0.5;  // how hard to brake when over the lateral limit
    constexpr double brake_cap = 0.6;   // max auto-brake the guard will add

    // Lateral acceleration the front can sustain = mu * g, scaled by how loaded the
    // front currently is vs. at rest (weight transfer under accel, bumps, and SCM
    // unloading all drop the front's grip). Open-loop (ratio=1) when no reference.
    const double g = GetSystem()->GetGravitationalAcceleration().Length();
    double load_ratio = 1.0;
    if (m_front_static_load > 1e-3) {
        load_ratio = FrontAxleNormalLoad(terrain) / m_front_static_load;
        load_ratio = std::clamp(load_ratio, 0.0, 1.5);
    }
    const double a_lat_max = mu * g * load_ratio;

    // Lateral acceleration demanded by the commanded steering at the current speed.
    const double v = m_vehicle->GetSpeed();
    const double delta = in.m_steering * max_steer;
    const double kappa = std::tan(delta) / wheelbase;  // path curvature
    const double a_lat = v * v * std::abs(kappa);

    if (a_lat_max <= 1e-6) {
        // Front essentially unloaded (e.g. wheelie): no cornering force available.
        in.m_throttle = 0.0;
        in.m_braking = std::max(in.m_braking, 0.3);
        return;
    }

    if (a_lat > a_lat_max) {
        // Over the friction limit: too fast to make this turn. Cut throttle, add brake
        // to shed speed, and clamp steering to the sharpest arc the front can hold
        // (steering past it just plows/understeers). This makes it slow down to turn.
        in.m_throttle = 0.0;
        const double over = a_lat / a_lat_max - 1.0;
        in.m_braking = std::max(in.m_braking, std::clamp(brake_gain * over, 0.0, brake_cap));

        const double kappa_feasible = a_lat_max / std::max(v * v, 1e-3);
        const double steer_feasible = std::atan(wheelbase * kappa_feasible) / max_steer;
        in.m_steering = std::clamp(in.m_steering, -steer_feasible, steer_feasible);
    } else {
        // Within the lateral limit: share the friction circle with longitudinal force.
        // The more of the lateral budget the turn uses, the less throttle is allowed,
        // so the front stays planted through the turn. rho in [0,1]; cap = sqrt(1-rho^2).
        const double rho = a_lat / a_lat_max;
        const double throttle_cap = std::sqrt(std::max(0.0, 1.0 - rho * rho));
        in.m_throttle = std::min(in.m_throttle, throttle_cap);
    }
}

void RobotRig::Synchronize(double time, chrono::vehicle::ChTerrain& terrain) {
    UpdateRockCollisionActivation();
    m_driver->Synchronize(time);
#ifdef AMD_UW_ENABLE_ROS2
    if (m_arm_bridge)
        m_arm_bridge->Synchronize(time, terrain);
#else
    if (m_arm)
        m_arm->Update(time);
#endif
    auto driver_inputs = m_driver->GetInputs();
    ApplyTractionGuard(driver_inputs, terrain);
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
                                 chrono::vehicle::ChTerrain& terrain) const {
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
