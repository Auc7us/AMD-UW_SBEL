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
#include "RosTrailerBridge.h"
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
                                   double height_probe_z,
                                   double vehicle_start_clearance,
                                   double seat_clearance,
                                   double settle_time,
                                   double step_size,
                                   const RockFieldConfig& rock_field_config) {
    m_rock_top_heights.clear();
    // Retained so a later harvest cycle can spawn its rocks the same way.
    m_terrain = &terrain;
    m_rock_mat = rock_mat;
    m_chrono_data_path = chrono_data_path;
    m_amd_uw_data_path = amd_uw_data_path;
    m_rock_field_config = rock_field_config;
    m_rocks = AddRockFields(GetSystem(), terrain, rock_mat, chrono_data_path, amd_uw_data_path, m_robot_index,
                            m_num_robots, height_probe_z, rock_field_config, &m_rock_top_heights,
                            m_harvest_cycle);

    const chrono::ChVector3d start_ground = InitialGroundPositionForRobot(m_robot_index, m_num_robots);
    const double start_x = start_ground.x();
    const double start_y = start_ground.y();
    const double start_z = terrain.GetHeight(chrono::ChVector3d(start_x, start_y, height_probe_z)) +
                           vehicle_start_clearance;
    const chrono::ChVector3d init_loc(start_x, start_y, start_z);
    const chrono::ChQuaternion<> init_rot =
        chrono::QuatFromAngleZ(InitialHeadingRadForRobot(m_robot_index, m_num_robots));

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
    // Settled per-wheel vertical tire load, tractor and trailer. TMeasy tires are
    // parameterised for a nominal load; these are the loads they actually carry, so
    // a gross mismatch (which makes the tire absurdly stiff for its duty) is visible.
    {
        int wheel_index = 0;
        for (const auto& axle : m_vehicle->GetAxles()) {
            for (const auto& wheel : axle->GetWheels()) {
                const auto& tire = wheel->GetTire();
                if (tire)
                    std::cout << "[RobotRig] rank " << m_rank << " settled load: tractor wheel " << wheel_index
                              << " Fz=" << std::abs(tire->ReportTireForce(&terrain).force.z()) << " N\n";
                ++wheel_index;
            }
        }
    }

    // Baselines for the trailer wheel anomaly probe, captured after settling.
    m_height_probe_z = height_probe_z;
    size_t total_wheels = 0;
    for (const auto& axle : m_vehicle->GetAxles())
        total_wheels += axle->GetWheels().size();
    for (const auto& axle : m_trailer->GetAxles())
        total_wheels += axle->GetWheels().size();
    m_wheel_sunk.assign(total_wheels, false);
    m_trailer_wheel_last_height.clear();
    m_trailer_wheel_static_fz.clear();
    for (const auto& axle : m_trailer->GetAxles()) {
        for (const auto& wheel : axle->GetWheels()) {
            const auto pos = wheel->GetSpindle()->GetPos();
            m_trailer_wheel_last_height.push_back(
                terrain.GetHeight(chrono::ChVector3d(pos.x(), pos.y(), m_height_probe_z)));
            const auto& tire = wheel->GetTire();
            m_trailer_wheel_static_fz.push_back(
                tire ? std::abs(tire->ReportTireForce(&terrain).force.z()) : 0.0);
            std::cout << "[RobotRig] rank " << m_rank << " settled load: trailer wheel "
                      << (m_trailer_wheel_static_fz.size() - 1) << " Fz=" << m_trailer_wheel_static_fz.back()
                      << " N\n";
        }
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
                chrono::vehicle::GetVehicleDataFile("LRV/Polaris_TMeasyTire.json"));
            // Render the configured tire OBJ while keeping TMeasy force-element
            // dynamics for tire-terrain interaction.
            m_vehicle->InitializeTire(tire, wheel, chrono::VisualizationType::MESH);
            tire->SetStepsize(m_tire_step_size);
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
            // Trailer-specific tire: a trailer wheel carries ~107 N against the
            // tractor's ~480 N, and TMeasy stiffness follows its nominal load, so a
            // single shared tire cannot suit both without being far too stiff for one.
            auto tire = chrono::vehicle::ReadTireJSON(
                chrono::vehicle::GetVehicleDataFile("LRV/Polaris_TMeasyTire_Trailer.json"));
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

namespace {

// Dump cycle geometry and timing.
//
// The bed is hinged about the trailer lateral axis and open at the rear, so a
// positive tilt of this size drops the rear lip well below the front and the load
// slides out. The angle MUST exceed the friction angle of the bed material, and the
// previous 40 deg did not: the bed is mu = 0.9, a rock slides only when
// tan(theta) > mu, and arctan(0.9) = 42.0 deg. At 40 deg (tan = 0.839) the load is
// below the threshold and is not supposed to move at all -- a dump that worked was
// a rock rolling or being nudged by the tilt, not sliding, which is why three ranks
// emptied and the fourth kept its rock sitting 0.28 m from the bed centre.
//
// 55 deg gives tan = 1.43 against mu = 0.9, a comfortable margin. Note gravity
// cancels out of tan(theta) > mu entirely, so lunar gravity never entered into it
// and this failed exactly as it would on Earth. If the bed material's friction
// changes, this angle has to be re-checked against arctan(mu).
constexpr double dump_bed_angle = 55.0 * chrono::CH_DEG_TO_RAD;
constexpr double dump_tailgate_angle = 95.0 * chrono::CH_DEG_TO_RAD;
// Slew rates. Slow enough that the bed and gate carry the load rather than
// launching it: at these rates the rear lip descends at well under 0.2 m/s.
constexpr double bed_slew_rate = 12.0 * chrono::CH_DEG_TO_RAD;       // ~3.3 s to full tilt
constexpr double tailgate_slew_rate = 60.0 * chrono::CH_DEG_TO_RAD;  // ~1.6 s to full swing
// Hold at full tilt so rocks have time to actually leave the tub.
constexpr double dump_dwell_time = 3.0;
// Bed footprint, in the bed's own frame, used to decide whether a rock is aboard.
// Matches the placement grid in RosArmBridge (bed floor ~1.0 x 1.2 m), with a little
// margin so a rock resting against a wall still counts as in the bed.
constexpr double bed_half_length = 0.7;
constexpr double bed_half_width = 0.8;
constexpr double bed_clear_height = 1.2;
// Stuck detector: throttle applied but not moving. See CheckStuck.
constexpr double stuck_speed = 0.08;          // m/s, below this counts as not moving
constexpr double stuck_dwell = 3.0;           // s of sim before it is called stuck
constexpr double stuck_report_period = 10.0;  // s of sim between repeat reports
// A stage is finished when its angle is this close to target.
constexpr double dump_angle_tol = 0.5 * chrono::CH_DEG_TO_RAD;

// Move `angle` toward `target` by at most `rate * dt`. Returns true once there.
bool SlewAngle(double& angle, double target, double rate, double dt) {
    const double step = rate * dt;
    if (std::abs(target - angle) <= std::max(step, dump_angle_tol)) {
        angle = target;
        return true;
    }
    angle += (target > angle) ? step : -step;
    return false;
}

}  // namespace

bool RobotRig::RequestTrailerDump() {
    if (!m_trailer_bed_motor || !m_trailer_tailgate_hinge)
        return false;
    if (m_dump_state != DumpState::IDLE && m_dump_state != DumpState::DONE)
        return false;

    m_dump_state = DumpState::OPENING_GATE;
    m_dump_stage_time = 0.0;

    // Record which rocks are riding in the bed, so the end of the cycle can say
    // whether they actually left it. Without this the cycle only proves the bed
    // moved: the motors reach their commanded angles and report success whether the
    // load tipped out, stayed put, or was flung somewhere it should not be.
    m_carried_rocks.clear();
    for (const auto& rock : m_rocks) {
        if (!RockIsInBed(rock))
            continue;
        m_carried_rocks.push_back(rock);

        // A rock that reaches home and comes to rest gets AUTO-SLEPT by Chrono, and a
        // sleeping body is excluded from the dynamics: it ignores gravity and it ignores
        // the bed tilting underneath it. The load then appears welded to the bed at full
        // incline. It cannot happen earlier in the mission -- a rock cannot fall asleep
        // while the rover is driving -- so it only ever shows up at the dump.
        //
        // Report the state before forcing it, because "asleep", "collision disabled" and
        // "still fixed by the arm" all look identical from outside (load does not move)
        // and each implicates different code.
        const bool was_sleeping = rock->IsSleeping();
        const bool was_fixed = rock->IsFixed();
        const bool had_collision = rock->IsCollisionEnabled();
        if (was_sleeping || was_fixed || !had_collision) {
            std::cout << "[RobotRig] rank " << m_rank << " dump: rock in bed was"
                      << (was_sleeping ? " ASLEEP" : "") << (was_fixed ? " FIXED" : "")
                      << (had_collision ? "" : " NON-COLLIDING")
                      << " -- it could not have tipped out. Forcing it dynamic.\n";
        }
        rock->SetSleepingAllowed(false);
        rock->SetSleeping(false);
        rock->SetFixed(false);
        rock->EnableCollision(true);
    }
    std::cout << "[RobotRig] rank " << m_rank << " dump requested: " << m_carried_rocks.size()
              << " rock(s) in the bed\n";
    return true;
}

// A rock counts as "in the bed" if it sits within the bed footprint in the bed's own
// frame, and above its floor. Bed local frame: x along the trailer, y across it.
bool RobotRig::RockIsInBed(const std::shared_ptr<chrono::ChBodyAuxRef>& rock) const {
    if (!m_trailer_bed || !rock)
        return false;
    const chrono::ChVector3d local =
        m_trailer_bed->GetRot().RotateBack(rock->GetPos() - m_trailer_bed->GetPos());
    return std::abs(local.x()) < bed_half_length && std::abs(local.y()) < bed_half_width &&
           local.z() > -0.3 && local.z() < bed_clear_height;
}

void RobotRig::AdvanceDumpCycle(double time) {
    if (!m_trailer_bed_motor || !m_trailer_tailgate_hinge)
        return;
    if (m_dump_state == DumpState::IDLE || m_dump_state == DumpState::DONE)
        return;

    // Slew against the real elapsed time so the cycle takes the same wall of sim
    // time regardless of step size.
    const double dt = (m_last_dump_time < 0.0) ? 0.0 : time - m_last_dump_time;
    m_last_dump_time = time;
    if (dt <= 0.0)
        return;

    switch (m_dump_state) {
        case DumpState::OPENING_GATE:
            // Gate first: tilting into a closed tailgate would just press the load
            // against it and then release it all at once when it finally opened.
            if (SlewAngle(m_tailgate_angle, dump_tailgate_angle, tailgate_slew_rate, dt)) {
                m_dump_state = DumpState::TILTING;
                m_dump_stage_time = 0.0;
            }
            break;
        case DumpState::TILTING:
            if (SlewAngle(m_bed_angle, dump_bed_angle, bed_slew_rate, dt)) {
                m_dump_state = DumpState::DWELL;
                m_dump_stage_time = 0.0;
                // Confirm the tilt actually drops the OPEN rear lip rather than the
                // closed front wall. Getting the motor axis sign wrong here does not
                // fail loudly -- it just presses the load against the front wall and
                // silently dumps nothing -- so report the two lip heights.
                const double half_x = 0.5;  // bed floor is 1.0 m along the trailer
                const auto& bed_rot = m_trailer_bed->GetRot();
                const double front_z = (m_trailer_bed->GetPos() + bed_rot.Rotate({half_x, 0.0, 0.0})).z();
                const double rear_z = (m_trailer_bed->GetPos() + bed_rot.Rotate({-half_x, 0.0, 0.0})).z();
                std::cout << "[RobotRig] rank " << m_rank << " bed at full tilt: front_lip_z=" << front_z
                          << " rear_lip_z=" << rear_z << " drop=" << (front_z - rear_z)
                          << " m (positive means the open rear is lower, which is correct)\n";
            }
            break;
        case DumpState::DWELL:
            m_dump_stage_time += dt;
            if (m_dump_stage_time >= dump_dwell_time) {
                m_dump_state = DumpState::LEVELING;
                m_dump_stage_time = 0.0;
            }
            break;
        case DumpState::LEVELING:
            if (SlewAngle(m_bed_angle, 0.0, bed_slew_rate, dt)) {
                m_dump_state = DumpState::CLOSING_GATE;
                m_dump_stage_time = 0.0;
            }
            break;
        case DumpState::CLOSING_GATE:
            // Gate last, so it latches against a level bed.
            if (SlewAngle(m_tailgate_angle, 0.0, tailgate_slew_rate, dt)) {
                m_dump_state = DumpState::DONE;
                m_dump_stage_time = 0.0;
                ReportDumpOutcome(time);
                // The load is on the ground: rotate the lane and put out the next set.
                StartNextHarvestCycle(time);
            }
            break;
        default:
            break;
    }

    m_trailer_bed_motor->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(m_bed_angle));
    m_trailer_tailgate_hinge->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(m_tailgate_angle));
}

void RobotRig::StartNextHarvestCycle(double time) {
    ++m_harvest_cycle;
    if (!m_terrain)
        return;

    // Rotate this rank's whole lane one step counter-clockwise and spawn a fresh set
    // of rocks on it. Everything -- rock line, drop point, builder station -- comes
    // from RankRayAngleRad(rank, N, cycle), so rotating is just advancing the index.
    const size_t first_new = m_rocks.size();
    auto new_rocks = AddRockFields(GetSystem(), *m_terrain, m_rock_mat, m_chrono_data_path, m_amd_uw_data_path,
                                   m_robot_index, m_num_robots, m_height_probe_z, m_rock_field_config,
                                   &m_rock_top_heights, m_harvest_cycle);

    // APPEND, never replace. The Python controllers track finished rocks by INDEX, so
    // existing indices have to keep meaning what they meant; new rocks simply arrive as
    // fresh indices the controllers have not completed yet. Rebuilding the vector would
    // silently mark the new rocks as already done.
    m_rocks.insert(m_rocks.end(), new_rocks.begin(), new_rocks.end());

    // Bodies added after the SolidWorks arm import do not register contacts unless the
    // collision system is rebound -- the same trap BuilderRig works around. Without
    // this the new rocks fall through the terrain and the gripper passes through them.
    GetSystem()->GetCollisionSystem()->BindAll();

    // The drop point moves with the lane, so the rover must be told where home is now.
    const chrono::ChVector3d home = InitialGroundPositionForRobot(m_robot_index, m_num_robots, m_harvest_cycle);
#ifdef AMD_UW_ENABLE_ROS2
    if (auto* ros_driver = dynamic_cast<RosControllerDriver*>(m_driver.get()))
        ros_driver->SetHomePosition(home);
#endif

    const double angle_deg = RankRayAngleRad(m_robot_index, m_num_robots, m_harvest_cycle) * chrono::CH_RAD_TO_DEG;
    std::cout << "[RobotRig] rank " << m_rank << " harvest cycle " << m_harvest_cycle << " at t=" << time
              << ": lane rotated to " << angle_deg << " deg, spawned " << (m_rocks.size() - first_new)
              << " rock(s), new drop point (" << home.x() << ", " << home.y() << "), "
              << m_rocks.size() << " rock(s) total in this rank\n";
}

void RobotRig::ReportDumpOutcome(double time) {
    std::cout << "[RobotRig] rank " << m_rank << " dump cycle complete at t=" << time << "\n";
    if (m_carried_rocks.empty()) {
        std::cout << "[RobotRig] rank " << m_rank
                  << " dump outcome: nothing was in the bed, so this proves only that the bed moved\n";
        return;
    }

    const chrono::ChVector3d bed_pos = m_trailer_bed ? m_trailer_bed->GetPos() : chrono::VNULL;
    int left = 0;
    int stuck = 0;
    for (size_t i = 0; i < m_carried_rocks.size(); ++i) {
        const auto& rock = m_carried_rocks[i];
        const bool in_bed = RockIsInBed(rock);
        const chrono::ChVector3d pos = rock->GetPos();
        const double range = (pos - bed_pos).Length();
        if (in_bed)
            ++stuck;
        else
            ++left;
        std::cout << "[RobotRig] rank " << m_rank << " dump outcome: rock " << i << " "
                  << (in_bed ? "STILL IN BED" : "left the bed") << " at (" << pos.x() << ", " << pos.y() << ", "
                  << pos.z() << "), " << range << " m from the bed\n";
    }
    std::cout << "[RobotRig] rank " << m_rank << " dump outcome: " << left << " of " << m_carried_rocks.size()
              << " rock(s) left the bed, " << stuck << " still aboard\n";
    m_carried_rocks.clear();
}

void RobotRig::DumpTrailerBed() {
    RequestTrailerDump();
}

void RobotRig::InitializeDriver() {
#ifdef AMD_UW_ENABLE_ROS2
    m_driver = std::make_unique<RosControllerDriver>(
        *m_vehicle, m_rank, m_rocks,
        InitialGroundPositionForRobot(m_robot_index, m_num_robots));
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
    m_trailer_bridge = std::make_unique<RosTrailerBridge>(m_rank, *this);
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

    ++m_guard_steps;
    if (a_lat > a_lat_max) {
        ++m_guard_limited_steps;
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
    CheckWheelSinkage(time, terrain);
    CheckStuck(time, terrain);
    CheckTrailerWheelAnomalies(time, terrain);
    AdvanceDumpCycle(time);
    m_driver->Synchronize(time);
#ifdef AMD_UW_ENABLE_ROS2
    if (m_trailer_bridge)
        m_trailer_bridge->Synchronize();
    if (m_arm_bridge)
        m_arm_bridge->Synchronize(time, terrain);
#else
    if (m_arm)
        m_arm->Update(time);
#endif
    auto driver_inputs = m_driver->GetInputs();
    m_last_raw_inputs = driver_inputs;
    ApplyTractionGuard(driver_inputs, terrain);
    m_last_guarded_inputs = driver_inputs;
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

void RobotRig::CheckStuck(double time, chrono::vehicle::ChTerrain& terrain) {
    // A rover being told to drive and not moving is the one failure that leaves no
    // trace anywhere: the controller sees its command accepted, the sim reports no
    // anomaly, and the rank simply never arrives. Everything needed to tell the
    // causes apart lives on this side, so dump it all in one report:
    //
    //  - guarded vs raw inputs: the traction guard cuts throttle and ADDS brake when
    //    it thinks the corner is over the friction limit, which looks like a seized
    //    axle from outside.
    //  - per-wheel Fz: a wheel at zero load is airborne (high-centred / tipped),
    //    while a wheel far above nominal means the tire is bottoming out.
    //  - per-wheel ground clearance: whether it is dug in rather than transiently low.
    //  - roll/pitch: whether it is simply stuck on a slope it cannot climb.
    const double speed = std::abs(m_vehicle->GetSpeed());
    const bool trying = m_last_guarded_inputs.m_throttle > 0.05;
    if (!trying || speed > stuck_speed) {
        m_stuck_since = -1.0;
        return;
    }
    if (m_stuck_since < 0.0) {
        m_stuck_since = time;
        return;
    }
    if (time - m_stuck_since < stuck_dwell || time - m_last_stuck_report < stuck_report_period)
        return;
    m_last_stuck_report = time;

    const auto pos = m_vehicle->GetChassisBody()->GetPos();
    const auto rot = m_vehicle->GetChassisBody()->GetRot();
    const auto rpy = rot.GetCardanAnglesXYZ();
    std::cout << "[RobotRig] rank " << m_rank << " STUCK for " << (time - m_stuck_since) << " s at t=" << time
              << ": pos=(" << pos.x() << ", " << pos.y() << ", " << pos.z() << ") speed=" << speed
              << " raw(str/thr/brk)=" << m_last_raw_inputs.m_steering << "/" << m_last_raw_inputs.m_throttle
              << "/" << m_last_raw_inputs.m_braking << " guarded=" << m_last_guarded_inputs.m_steering << "/"
              << m_last_guarded_inputs.m_throttle << "/" << m_last_guarded_inputs.m_braking
              << " roll=" << (rpy.x() * chrono::CH_RAD_TO_DEG) << " pitch=" << (rpy.y() * chrono::CH_RAD_TO_DEG)
              << " deg\n";
    // Rocks close enough to be in the way. Rocks are small (0.2 scale) but they are
    // rigid and collidable within 12 m, and a rig can be stopped by one wedged under
    // the chassis or against a wheel while every other reading looks normal. Dumped
    // piles matter as much as targets: the drop point is on the route out to the next
    // lane, so a rover can drive into the load it left behind on the previous cycle.
    {
        const auto tractor_pos = m_vehicle->GetChassisBody()->GetPos();
        const auto trailer_pos =
            (m_trailer && m_trailer->GetChassis()) ? m_trailer->GetChassis()->GetPos() : tractor_pos;
        for (size_t i = 0; i < m_rocks.size(); ++i) {
            const auto rpos = m_rocks[i]->GetPos();
            const double d_tractor = std::sqrt(PlanarDistance2(rpos, tractor_pos));
            const double d_trailer = std::sqrt(PlanarDistance2(rpos, trailer_pos));
            const double nearest = std::min(d_tractor, d_trailer);
            if (nearest < 5.0) {
                std::cout << "[RobotRig] rank " << m_rank << " STUCK   rock " << i << " at (" << rpos.x()
                          << ", " << rpos.y() << ", " << rpos.z() << ") is " << d_tractor
                          << " m from the tractor, " << d_trailer << " m from the trailer, collision="
                          << (m_rocks[i]->IsCollisionEnabled() ? "on" : "off") << "\n";
            }
        }
    }

    // Articulation angle between tractor and trailer. A rig that PIVOTS instead of
    // driving -- tens of degrees of yaw for a metre of travel with zero steering
    // commanded -- is being held at the rear, and a jackknifed trailer does exactly
    // that: it drags sideways, swings the tractor round, and blocks any steering
    // further into the fold no matter how much throttle is applied. Nothing else in
    // this report can see it, because the trailer has no driven wheels and its tires
    // report normal loads while skidding.
    if (m_trailer && m_trailer->GetChassis()) {
        const auto tractor_fwd = m_vehicle->GetChassisBody()->GetRot().GetAxisX();
        const auto trailer_fwd = m_trailer->GetChassis()->GetBody()->GetRot().GetAxisX();
        const double hitch_deg =
            std::atan2(tractor_fwd.x() * trailer_fwd.y() - tractor_fwd.y() * trailer_fwd.x(),
                       tractor_fwd.x() * trailer_fwd.x() + tractor_fwd.y() * trailer_fwd.y()) *
            chrono::CH_RAD_TO_DEG;
        std::cout << "[RobotRig] rank " << m_rank << " STUCK   hitch articulation=" << hitch_deg
                  << " deg (0 = trailer straight behind; large = jackknifed)\n";
        int t_index = 0;
        for (const auto& axle : m_trailer->GetAxles()) {
            for (const auto& wheel : axle->GetWheels()) {
                const auto wpos = wheel->GetSpindle()->GetPos();
                const double ground =
                    terrain.GetHeight(chrono::ChVector3d(wpos.x(), wpos.y(), m_height_probe_z));
                const auto& tire = wheel->GetTire();
                const double fz = tire ? std::abs(tire->ReportTireForce(&terrain).force.z()) : 0.0;
                std::cout << "[RobotRig] rank " << m_rank << " STUCK   trailer wheel " << t_index
                          << " Fz=" << fz << " N clearance=" << (wpos.z() - ground)
                          << " m omega=" << wheel->GetState().omega << " rad/s\n";
                ++t_index;
            }
        }
    }

    // Driveline state: an engine at idle with the throttle open, or a transmission
    // that never picked a gear, both present as "throttle applied, nothing happens".
    if (auto powertrain = m_vehicle->GetPowertrainAssembly()) {
        std::cout << "[RobotRig] rank " << m_rank << " STUCK   engine=" << powertrain->GetEngine()->GetMotorSpeed()
                  << " rad/s torque=" << powertrain->GetEngine()->GetOutputMotorshaftTorque()
                  << " Nm gear=" << powertrain->GetTransmission()->GetCurrentGear() << "\n";
    }

    int index = 0;
    for (const auto& axle : m_vehicle->GetAxles()) {
        for (const auto& wheel : axle->GetWheels()) {
            const auto wpos = wheel->GetSpindle()->GetPos();
            const double ground = terrain.GetHeight(chrono::ChVector3d(wpos.x(), wpos.y(), m_height_probe_z));
            const auto& tire = wheel->GetTire();
            const double fz = tire ? std::abs(tire->ReportTireForce(&terrain).force.z()) : 0.0;
            // omega is the discriminator the first version of this report lacked. All
            // wheels loaded, ground flat, brake off and guard idle still leaves two
            // very different failures: wheels TURNING while the chassis does not move
            // is a traction failure, wheels NOT turning is the driveline delivering no
            // torque. The load and clearance numbers look identical either way.
            const double omega = tire ? wheel->GetState().omega : 0.0;
            std::cout << "[RobotRig] rank " << m_rank << " STUCK   tractor wheel " << index << " Fz=" << fz
                      << " N clearance=" << (wpos.z() - ground) << " m omega=" << omega << " rad/s (rim speed "
                      << (omega * 0.4089) << " m/s)\n";
            ++index;
        }
    }
}

void RobotRig::CheckWheelSinkage(double time, chrono::vehicle::ChTerrain& terrain) {
    // TMeasy tires carry NO collision geometry -- nothing physically stops a wheel from
    // ending up below the surface. The tire only asks the terrain for a height under its
    // contact patch and turns the deflection into a force, so if that force is too small
    // for the load the wheel simply descends through the ground, and if GetHeight's ray
    // misses (it returns 0, and this site sits metres above z=0) it descends with no
    // force at all.
    //
    // A healthy spindle rides ~0.35-0.41 m above ground (unloaded radius 0.4089 m).
    // Report on the way in and again on the way out, so a transient dip is
    // distinguishable from a rover that is stuck high-centred.
    if (m_sink_reports >= 12)
        return;

    constexpr double sunk_clearance = 0.15;   // spindle this close to ground = half buried
    constexpr double recovered_clearance = 0.25;

    size_t index = 0;
    auto check = [&](const std::shared_ptr<chrono::vehicle::ChWheel>& wheel, const char* which) {
        const auto pos = wheel->GetSpindle()->GetPos();
        const double ground = terrain.GetHeight(chrono::ChVector3d(pos.x(), pos.y(), m_height_probe_z));
        const double clearance = pos.z() - ground;
        if (index >= m_wheel_sunk.size())
            return;
        const bool sunk = clearance < sunk_clearance;
        if (sunk && !m_wheel_sunk[index]) {
            m_wheel_sunk[index] = true;
            ++m_sink_reports;
            std::cout << "[RobotRig] rank " << m_rank << " " << which << " wheel " << index
                      << " SUNK at t=" << time << ": spindle z=" << pos.z() << " ground z=" << ground
                      << " clearance=" << clearance << " m (tire radius 0.409)"
                      << (std::abs(ground) < 1e-9 ? "  [ground read exactly 0 -- ray MISSED]" : "") << "\n";
        } else if (!sunk && m_wheel_sunk[index] && clearance > recovered_clearance) {
            m_wheel_sunk[index] = false;
            ++m_sink_reports;
            std::cout << "[RobotRig] rank " << m_rank << " " << which << " wheel " << index
                      << " recovered at t=" << time << ": clearance=" << clearance << " m\n";
        }
        ++index;
    };

    for (const auto& axle : m_vehicle->GetAxles())
        for (const auto& wheel : axle->GetWheels())
            check(wheel, "TRACTOR");
    for (const auto& axle : m_trailer->GetAxles())
        for (const auto& wheel : axle->GetWheels())
            check(wheel, "TRAILER");
}

void RobotRig::CheckTrailerWheelAnomalies(double time, chrono::vehicle::ChTerrain& terrain) {
    // The trailer runs TMeasy tires, which are force elements with no collision
    // geometry: each one asks the terrain for a height and a normal under its
    // contact patch and turns the resulting deflection into a force. That makes two
    // failure modes possible that leave no contact trace at all.
    //
    // 1. RigidTerrain::GetHeight ray-casts and returns 0 when the ray MISSES, and
    //    this site sits several metres above z=0. One missed ray therefore reports
    //    the ground as metres below the wheel, and the step after it reports it back
    //    at the surface -- a deflection step that becomes an enormous single-wheel
    //    force. Patch seams and patch edges are where rays go missing.
    // 2. A tire parameterised for a load far above its actual lunar wheel load is
    //    very stiff relative to what it carries, so a small deflection error is a
    //    large force.
    //
    // Both look identical from the outside: one wheel suddenly launches. Report the
    // height discontinuity and the force spike separately so they can be told apart.
    if (m_trailer_wheel_last_height.empty() || m_trailer_anomaly_reports >= 20)
        return;

    constexpr double height_jump_tol = 0.5;   // m between consecutive steps
    constexpr double force_spike_factor = 8.0;  // multiple of the settled load
    constexpr double force_spike_floor = 2000.0;  // N, for near-unloaded wheels

    size_t index = 0;
    for (const auto& axle : m_trailer->GetAxles()) {
        for (const auto& wheel : axle->GetWheels()) {
            if (index >= m_trailer_wheel_last_height.size())
                return;
            const char* side = (index % 2 == 0) ? "LEFT" : "RIGHT";
            const auto pos = wheel->GetSpindle()->GetPos();
            const double height =
                terrain.GetHeight(chrono::ChVector3d(pos.x(), pos.y(), m_height_probe_z));
            const double previous = m_trailer_wheel_last_height[index];
            m_trailer_wheel_last_height[index] = height;

            if (std::abs(height - previous) > height_jump_tol) {
                ++m_trailer_anomaly_reports;
                std::cout << "[RobotRig] rank " << m_rank << " TRAILER " << side
                          << " wheel terrain-height JUMP at t=" << time << ": " << previous << " -> " << height
                          << " m at (" << pos.x() << ", " << pos.y() << ")"
                          << (std::abs(height) < 1e-9 ? "  [returned exactly 0 -- ray MISSED the terrain]" : "")
                          << "\n";
            }

            const auto& tire = wheel->GetTire();
            if (tire) {
                const double fz = std::abs(tire->ReportTireForce(&terrain).force.z());
                const double reference = std::max(force_spike_floor,
                                                  force_spike_factor * m_trailer_wheel_static_fz[index]);
                if (fz > reference) {
                    ++m_trailer_anomaly_reports;
                    std::cout << "[RobotRig] rank " << m_rank << " TRAILER " << side
                              << " wheel vertical force SPIKE at t=" << time << ": " << fz << " N (settled "
                              << m_trailer_wheel_static_fz[index] << " N) at (" << pos.x() << ", " << pos.y()
                              << ")\n";
                }
            }
            ++index;
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
                                << " tire_force_sum=" << tire_force_sum << " tire_force_z=" << tire_force_z
                                << " step_rtf=" << m_vehicle->GetStepRTF() << "\n";
}

}  // namespace amd_uw
