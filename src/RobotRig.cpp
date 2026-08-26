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

// Divergence tripwire thresholds. See RobotRig::CheckDivergence.
//
// The point of the WARN tier is that NaN is a useless place to start looking: by
// the time a value is NaN it has propagated through every constraint the body
// touches, and every rank body reads NaN within a few steps. The blow-up is only
// localised while it is still FINITE, so the tripwire has to fire on absurd-but-
// finite motion and name the body then. 60 m/s is ~10x anything on this site (the
// rover cruises at 5) and 200 rad/s is ~500x a wheel at speed.
constexpr double divergence_speed_warn = 60.0;      // m/s
constexpr double divergence_omega_warn = 200.0;     // rad/s
constexpr double divergence_scan_period = 0.05;     // sim s between scans
constexpr int divergence_max_reports = 6;

// Dumped-rock freeze. See RobotRig::UpdateDumpedRockFreeze.
constexpr double dumped_rock_settle_speed = 0.05;   // m/s below which it counts as at rest
constexpr double dumped_rock_settle_dwell = 1.0;    // s it must stay there before freezing

bool IsFinite(const chrono::ChVector3d& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

bool IsFinite(const chrono::ChQuaternion<>& q) {
    return std::isfinite(q.e0()) && std::isfinite(q.e1()) && std::isfinite(q.e2()) && std::isfinite(q.e3());
}

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

    // Spawn radius, not the drop ring: the two are separate so the rig does not start
    // parked against its own builder. See robot_spawn_radius.
    const chrono::ChVector3d start_ground = InitialSpawnPositionForRobot(m_robot_index, m_num_robots);
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
        // Sized in RobotLayout.h: covers the rock plus margin and not a square metre
        // more, because every extra node is re-probed against a growing hash map on
        // every step of the run.
        for (auto& rock : m_rocks)
            scm->AddActiveDomain(rock, scm_rock_domain_center, scm_rock_domain_dims);
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

    // ENGAGE the differential locks. Setting the locking limits in Polaris_4WD.json
    // is necessary but on its own does nothing at all.
    //
    // ChShaftsDriveline4WD builds each differential lock as a ChShaftsClutch and
    // then calls SetModulation(0) on it -- "By default, unlocked". The JSON
    // "Axle/Central Differential Locking Limit" only sets that clutch's torque
    // CAPACITY via SetTorqueLimit; modulation is the engagement, and at 0 the
    // clutch transmits nothing no matter how large the limit is. Raising the limit
    // alone therefore leaves the differentials fully open, which is what a wheel
    // spinning at 31 rad/s beside its own axle-mate at 0.02 rad/s means.
    //
    // Both halves are needed: modulation 1 engages the clutch, and the raised limit
    // stops it slipping again at Chrono's default 100 Nm.
    //
    // -1 locks both driven axles. LockCentralDifferential ignores its first
    // argument. Locked for the whole run: these are slow rovers on loose regolith
    // in lunar gravity, where every wheel load is ~6x smaller than on Earth, and
    // real lunar rovers sidestep the problem entirely by driving each wheel
    // independently. The scrub a locked axle causes while turning costs far less
    // than being stranded.
    if (auto* driveline = m_vehicle->GetDriveline().get()) {
        driveline->LockAxleDifferential(-1, true);
        driveline->LockCentralDifferential(-1, true);
    }

    for (auto& axle : m_vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            // Lugged RIGID MESH tyre, not TMeasy. On deformable terrain a TMeasy wheel
            // is a force element with no collision geometry, so SCM's rays hit nothing
            // there: the wheel rides the deformed surface but never cuts a rut and never
            // earns sinkage resistance. See Polaris_LuggedTire.json for why bolting a
            // shape onto TMeasy instead would double-count the support.
            auto tire = chrono::vehicle::ReadTireJSON(
                chrono::vehicle::GetVehicleDataFile("LRV/Polaris_LuggedTire.json"));
            // Render the smooth tyre OBJ; the lugs live in the collision mesh, which is
            // what works the soil.
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
            // One tyre now serves both, where TMeasy needed two. The split existed
            // because TMeasy's vertical stiffness follows its NOMINAL LOAD, and a
            // trailer wheel carries ~107 N against the tractor's ~480 N -- no single
            // TMeasy tyre suited both. A rigid mesh tyre has no nominal load: its
            // stiffness is the contact material's, so the same geometry is correct at
            // any wheel load, and both tyres are the same 0.4089 x 0.30 anyway.
            auto tire = chrono::vehicle::ReadTireJSON(
                chrono::vehicle::GetVehicleDataFile("LRV/Polaris_LuggedTire.json"));
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

    // Open dumping tub (floor + the +x front wall + both +/-y side walls, open at the
    // REAR -x end), a DYNAMIC body carried by the trailer through a lateral-axis
    // revolute motor held flat. Unlike the old fixed/teleported plate, a jointed dynamic
    // bed has real velocity, so friction keeps placed rocks aboard while driving.
    // Extents come from RobotLayout so the sensor rank's visual-only copy of this bed
    // is built from the same numbers. See the trailer_bed_* constants there.
    const double ex = trailer_bed_floor_x;  // footprint: x along the trailer, y across
    const double ey = trailer_bed_floor_y;
    const double wall_h = trailer_bed_wall_height;
    const double t = trailer_bed_thickness;

    m_trailer_bed = chrono_types::make_shared<chrono::ChBody>();
    m_trailer_bed->SetName("trailer_bed");
    m_trailer_bed->SetPos(bed_pos);
    m_trailer_bed->SetRot(chassis->GetRot());
    const double mass = 30.0;
    m_trailer_bed->SetMass(mass);
    m_trailer_bed->SetInertiaXX(chrono::ChVector3d(mass / 12.0 * (ey * ey + wall_h * wall_h),
                                                   mass / 12.0 * (ex * ex + wall_h * wall_h),
                                                   mass / 12.0 * (ex * ex + ey * ey)));

    // Per-rank colour, matched to this rank's builder. See RankColor.
    const chrono::ChColor bed_color = RankColor(m_robot_index);

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
    // Floor, +x front wall, +/-y side walls. The rear (-x) is open for the tailgate.
    for (const auto& box : TrailerBedBoxes()) {
        add_box(m_trailer_bed, box.size.x(), box.size.y(), box.size.z(), box.center.x(), box.center.y(),
                box.center.z());
    }
    m_trailer_bed->EnableCollision(true);
    GetSystem()->AddBody(m_trailer_bed);

    // Hinged REAR tailgate (-x): a separate dynamic body, connected to the bed with a
    // revolute joint about the trailer lateral (Y) axis, sitting on the rear pour lip.
    const chrono::ChVector3d tailgate_center_local(-(trailer_bed_half_x + t / 2.0), 0.0,
                                                   t / 2.0 + wall_h / 2.0);
    const chrono::ChVector3d tailgate_hinge_local(-(trailer_bed_half_x + t / 2.0), 0.0, t / 2.0);
    const chrono::ChVector3d tailgate_pos = bed_pos + chassis->GetRot().Rotate(tailgate_center_local);

    m_trailer_tailgate = chrono_types::make_shared<chrono::ChBody>();
    m_trailer_tailgate->SetName("trailer_tailgate");
    m_trailer_tailgate->SetPos(tailgate_pos);
    m_trailer_tailgate->SetRot(chassis->GetRot());
    const double tailgate_mass = 5.0;
    m_trailer_tailgate->SetMass(tailgate_mass);
    // Gate box is (t, ey, wall_h): it runs ACROSS the trailer, closing the open rear end.
    m_trailer_tailgate->SetInertiaXX(
        chrono::ChVector3d(tailgate_mass / 12.0 * (ey * ey + wall_h * wall_h),
                           tailgate_mass / 12.0 * (t * t + wall_h * wall_h),
                           tailgate_mass / 12.0 * (t * t + ey * ey)));
    {
        const auto gate = TrailerTailgateBox();
        add_box(m_trailer_tailgate, gate.size.x(), gate.size.y(), gate.size.z(), gate.center.x(), gate.center.y(),
                gate.center.z());
    }
    m_trailer_tailgate->EnableCollision(true);
    GetSystem()->AddBody(m_trailer_tailgate);

    // Tailgate hinge as a rotation motor held at angle 0 -> the gate stays CLOSED
    // (a free revolute here just dangled open, since a bottom-hinged flap is
    // unstable upright under gravity). Motor turns about frame Z, so rotate +90 deg
    // about X to put Z on the trailer's -Y axis: about -Y, the gate's top (+z) swings
    // toward -x, i.e. the gate falls REARWARD and outward into a chute. About +Y it
    // would swing forward into the tub.
    const chrono::ChQuaternion<> tailgate_hinge_rot = chassis->GetRot() * chrono::QuatFromAngleX(chrono::CH_PI_2);
    const chrono::ChVector3d tailgate_hinge_pos = bed_pos + chassis->GetRot().Rotate(tailgate_hinge_local);
    m_trailer_tailgate_hinge = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
    m_trailer_tailgate_hinge->SetName("trailer_tailgate_hinge");
    m_trailer_tailgate_hinge->Initialize(m_trailer_tailgate, m_trailer_bed,
                                         chrono::ChFramed(tailgate_hinge_pos, tailgate_hinge_rot));
    m_trailer_tailgate_hinge->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.0));
    GetSystem()->AddLink(m_trailer_tailgate_hinge);

    // Revolute motor about the chassis lateral (-Y) axis, positioned ON THE REAR POUR
    // LIP rather than at the tub centre. Held at 0 => bed stays flat.
    //
    // The axis placement matters as much as its direction. Hinge at the tub centre and
    // the rear lip sweeps forward as d*cos(theta), so at the 55 deg dump angle a lip
    // 0.5 m behind the centre ends up 0.29 m behind it -- the pour line walks under the
    // trailer as the tub rises. Hinging ON the lip leaves it fixed in space: the front
    // of the tub lifts, the floor slopes back, and the load pours over a pour line that
    // stays put behind the tailgate.
    //
    // -Y, not +Y: about -Y a positive angle lifts the +x front, tipping the floor toward
    // the open rear. About +Y the same angle would tip it into the closed front wall.
    const chrono::ChQuaternion<> frame_rot = chassis->GetRot() * chrono::QuatFromAngleX(chrono::CH_PI_2);
    const chrono::ChVector3d bed_hinge_pos =
        bed_pos + chassis->GetRot().Rotate(chrono::ChVector3d(-trailer_bed_half_x, 0.0, 0.0));
    m_trailer_bed_motor = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
    m_trailer_bed_motor->Initialize(m_trailer_bed, chassis, chrono::ChFramed(bed_hinge_pos, frame_rot));
    m_trailer_bed_motor->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.0));
    GetSystem()->AddLink(m_trailer_bed_motor);
}

namespace {

// Dump cycle geometry and timing.
//
// The bed is hinged about the trailer lateral axis, on the open REAR lip, so a
// positive tilt of this size lifts the front and the load slides out backwards
// over that lip. The angle MUST exceed the friction angle of the bed material, and the
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
// The tub is centred on the trailer (trailer_bed_center_y = 0), so this is measured
// about the centreline, with margin so a rock resting against a side wall still counts.
constexpr double bed_half_width = 0.5 * trailer_bed_floor_y + 0.2;
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
    return std::abs(local.x()) < bed_half_length &&
           std::abs(local.y() - trailer_bed_center_y) < bed_half_width && local.z() > -0.3 &&
           local.z() < bed_clear_height;
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
                const double half_x = trailer_bed_half_x;
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

    // Step this rank's COLLECTOR lane counter-clockwise and spawn a fresh set of rocks
    // on it. Rock line and drop point both come from HarvestLaneAngleRad(rank, N, cycle),
    // so stepping the lane is just advancing the index. The builder's course does not
    // move with it -- the lane steps ALONG that course; see HarvestDropSlot.
    const size_t first_new = m_rocks.size();
    auto new_rocks = AddRockFields(GetSystem(), *m_terrain, m_rock_mat, m_chrono_data_path, m_amd_uw_data_path,
                                   m_robot_index, m_num_robots, m_height_probe_z, m_rock_field_config,
                                   &m_rock_top_heights, m_harvest_cycle);

    // APPEND, never replace. The Python controllers track finished rocks by INDEX, so
    // existing indices have to keep meaning what they meant; new rocks simply arrive as
    // fresh indices the controllers have not completed yet. Rebuilding the vector would
    // silently mark the new rocks as already done.
    m_rocks.insert(m_rocks.end(), new_rocks.begin(), new_rocks.end());

    // Bodies added after the SolidWorks arm import do not register contacts unless
    // they are bound into the collision system -- the same trap BuilderRig works
    // around. Without this the new rocks fall through the terrain and the gripper
    // passes through them.
    //
    // Bind ONLY the new rocks. This used to call BindAll(), which rebuilds the
    // collision state of every body in the rank -- while the rig is sitting in
    // resting contact with the terrain, the bed, and whatever it just dumped. Under
    // SMC a contact that is re-detected with its existing penetration delivers the
    // full penalty force in a single step rather than the force it had been
    // carrying, so BindAll was an impulse applied to the whole rank at the one
    // moment per cycle when it was guaranteed to be loaded. Rank 4 in the 3 h run
    // went non-finite ~0.4 s of sim time after this line. That is correlation, not
    // proof -- but a whole-system rebind was never needed here, since the only
    // unbound bodies are the ones this function just created.
    for (size_t i = first_new; i < m_rocks.size(); ++i)
        GetSystem()->GetCollisionSystem()->BindItem(m_rocks[i]);

    // On deformable terrain the new rocks also need their own SCM active domains. Being
    // bound into the collision system only makes them visible to ray casts that are
    // actually fired, and SCM fires rays only from nodes inside some active domain -- so
    // without this a later cycle's rocks spawn on the surface and sink straight through it.
    // The cycle-0 rocks get theirs in InitializeOnTerrain; these are the ones that follow.
    if (auto* scm = dynamic_cast<chrono::vehicle::SCMTerrain*>(m_terrain)) {
        for (size_t i = first_new; i < m_rocks.size(); ++i)
            scm->AddActiveDomain(m_rocks[i], scm_rock_domain_center, scm_rock_domain_dims);
    }

    // DO NOT bind these into the VSG scene from here -- it segfaults. VSG needs new
    // geometry compiled and merged between frames (the LoadOperation/Merge pattern in
    // ChVisualSystemVSG.cpp); BindBody does neither, so it is only safe before
    // Initialize(). Doing it from the physics loop races the render thread and killed
    // a 4-rank run. These rocks are still simulated and still visible to the sensor
    // camera, just absent from VSG. Proper fix: pre-create every cycle's rocks before
    // Initialize() and move them into place, as SynRockAgent does for its zombies.

    // The drop point moves with the lane, so the rover must be told where home is now.
    const chrono::ChVector3d home = InitialGroundPositionForRobot(m_robot_index, m_num_robots, m_harvest_cycle);
#ifdef AMD_UW_ENABLE_ROS2
    if (auto* ros_driver = dynamic_cast<RosControllerDriver*>(m_driver.get()))
        ros_driver->SetHomePosition(home);
#endif

    const double angle_deg =
        HarvestLaneAngleRad(m_robot_index, m_num_robots, m_harvest_cycle) * chrono::CH_RAD_TO_DEG;
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

    // Hand the rocks that made it out to the settle-then-freeze watcher. A dumped
    // rock is awake and on a slope, so it creeps and rolls away from the pile the
    // builder is meant to work from. Rocks look still at the START of the demo only
    // because Chrono has put them to sleep and nothing has disturbed them -- that is
    // not a freeze, and it does not survive being tipped out of a bed.
    for (const auto& rock : m_carried_rocks) {
        if (!RockIsInBed(rock))
            m_settling_rocks.push_back(rock);
    }
    m_carried_rocks.clear();
}

// Freeze a dumped rock once it stops, and let the builder wake it again.
//
// Mirrors what LrvArm does to its own target: SetFixed(true) with collision left
// ON, so the rock is a solid object that simply cannot be moved by gravity, soil,
// or a passing wheel. The freeze waits for the rock to come to rest first --
// freezing mid-roll would leave it hanging at whatever angle it happened to be at.
void RobotRig::UpdateDumpedRockFreeze(double time) {
    for (auto it = m_settling_rocks.begin(); it != m_settling_rocks.end();) {
        const auto& rock = *it;
        if (!rock || rock->IsFixed()) {
            it = m_settling_rocks.erase(it);
            continue;
        }
        const double speed = VecNorm(rock->GetPosDt());
        if (!std::isfinite(speed)) {
            it = m_settling_rocks.erase(it);
            continue;
        }
        if (speed > dumped_rock_settle_speed) {
            m_rock_still_since.erase(rock.get());
            ++it;
            continue;
        }
        auto since = m_rock_still_since.find(rock.get());
        if (since == m_rock_still_since.end()) {
            m_rock_still_since[rock.get()] = time;
            ++it;
            continue;
        }
        if (time - since->second < dumped_rock_settle_dwell) {
            ++it;
            continue;
        }

        rock->SetFixed(true);
        rock->EnableCollision(true);
        const auto p = rock->GetPos();
        std::cout << "[RobotRig] rank " << m_rank << " delivered rock FROZEN at t=" << time << " at (" << p.x()
                  << ", " << p.y() << ", " << p.z() << "); it stays put until the builder's gripper takes it\n";
        m_delivered_rocks.push_back(rock);
        m_rock_still_since.erase(rock.get());
        it = m_settling_rocks.erase(it);
    }
}

// A delivered rock is FEEDSTOCK, and feedstock stays where it was put.
//
// This used to release a frozen rock again -- unfixing it when its contact force rose
// above its own resting load, or as a backstop when any builder body came within 3 m --
// on the theory that a builder driving into a dumped pile should not hit immovable
// objects. That theory belonged to the version where the pile was in the builder's way.
// Now the pile IS the builder's larder: it parks beside it deliberately, and both
// triggers fire exactly then. The proximity one fires as the hull arrives on station,
// and the force one fires as the gripper's own fingers close on the rock -- so the rock
// would be unfixed and rolling off down the slope at the precise moment the arm is
// trying to take hold of it.
//
// The invariant is the seed heap's: every rock is fixed, and the only dynamic one is
// the one the gripper has locked. LrvArm::TryLockRock does that unfixing, and
// BuilderArmRosBridge re-fixes the rock once it has been laid.
std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> RobotRig::GetDeliveredRocks() const {
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> out;
    out.reserve(m_delivered_rocks.size());
    for (const auto& rock : m_delivered_rocks) {
        if (rock)
            out.push_back(rock);
    }
    return out;
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
    // First, before anything reads the state: a rank whose physics has blown up
    // must say so. Everything below this line silently no-ops on NaN.
    CheckDivergence(time);
    UpdateRockCollisionActivation();
    UpdateDumpedRockFreeze(time);
    CheckWheelSinkage(time, terrain);
    CheckStuck(time, terrain);
    CheckTrailerWheelAnomalies(time, terrain);
    ApplySteeringStops(time);
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
    UpdateAxleDifferentialLock(driver_inputs.m_steering);
    m_vehicle->Synchronize(time, driver_inputs, terrain);
    m_trailer->Synchronize(time, driver_inputs, terrain);
}

// Lock the axle differentials for straight running, release them to turn.
//
// Locked all the time is too blunt. A locked axle forces both wheels to the same
// speed while a turn needs them to travel different distances, so the tires scrub
// against each other; at full steering on a slope that scrub bogged the engine to
// 3.4 rad/s (33 rpm) in gear 1 and the rig stopped just as dead as it used to with
// a wheel spinning -- all four wheels turning together at 0.18 rad/s, going
// nowhere. Unlocked all the time is the original fault: an open differential sends
// every newton-metre to the least-loaded wheel, which at lunar gravity breaks
// traction almost immediately.
//
// So lock where locking pays -- hauling in a straight line, which is where the
// runaway wheel always appeared -- and release before the scrub can build. The two
// thresholds are hysteresis; a single one would chatter the clutch on and off
// around the boundary. The centre differential stays locked throughout: it splits
// front to rear, where the wheels travel the same distance through a turn, so it
// costs no scrub.
void RobotRig::UpdateAxleDifferentialLock(double steering) {
    constexpr double lock_below = 0.25;      // |steering| under this: lock
    constexpr double release_above = 0.35;   // over this: unlock, but only at speed
    constexpr double scrub_speed = 1.0;      // below this, scrub is irrelevant

    // Releasing on STEERING ALONE was wrong, and it broke the turnaround.
    //
    // Pure pursuit's hard-turn branch commands exactly 0.6 steering, which sailed
    // past the 0.35 release threshold and unlocked the differentials for the whole
    // post-dump turnaround -- a manoeuvre done at ~0 m/s, which is precisely when
    // an open differential dumps every newton-metre into the least-loaded wheel and
    // the rig stops moving. A rover was seen pirouetting for 38 s, covering 0.13 m,
    // at full throttle with the locks released.
    //
    // Scrub is a function of SPEED, not steer angle: two wheels forced to the same
    // speed only fight each other when they are actually covering ground, and the
    // energy involved scales with how fast. Standing still there is nothing to
    // scrub. So release only when the rover is both turning hard AND moving --
    // otherwise stay locked, because at low speed traction is the only thing that
    // matters.
    const double magnitude = std::abs(steering);
    const double speed = std::abs(m_vehicle->GetSpeed());

    bool want_locked;
    if (speed < scrub_speed) {
        want_locked = true;
    } else {
        want_locked = m_axle_diff_locked ? (magnitude <= release_above) : (magnitude < lock_below);
    }
    if (want_locked == m_axle_diff_locked)
        return;
    m_axle_diff_locked = want_locked;
    if (auto* driveline = m_vehicle->GetDriveline().get())
        driveline->LockAxleDifferential(-1, want_locked);
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
        // A FROZEN rock is delivered feedstock: it is fixed, sitting at the drop point,
        // waiting for the builder. This loop must not touch it. Deactivation exists to
        // keep distant DYNAMIC rocks out of the broadphase, and a fixed body is already
        // free -- but the rover drives away from its own drop point on the very next
        // cycle, so the distance test would switch off collision on exactly the pile the
        // gripper is about to close on, and the fingers would pass straight through it.
        if (rock->IsFixed())
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

// Name the body that is diverging, WHILE IT IS STILL FINITE.
//
// This exists because a rank that goes NaN currently dies without a word, and I
// have spent three sessions guessing at the mechanism from the silence. Every
// other detector in this file fails open on NaN -- `NaN > threshold` and
// `std::abs(NaN - ref) > 0.25` are both FALSE, so CheckStuck early-returns and the
// arm-base DRIFT check goes quiet at exactly the moment it should shout. Worse,
// once a value is NaN it has already propagated through every constraint the body
// touches and every body in the rank reads NaN within a few steps, so the NaN
// itself cannot tell you where it started.
//
// So there are two tiers. The WARN tier fires on absurd-but-FINITE motion, which
// is the only window in which the blow-up is still localised to one body -- that
// is the report that actually names a culprit. The FATAL tier catches the NaN
// itself and dumps enough state to reconstruct what the rig was doing.
//
// Both report the links touching the offending body and their reaction wrenches,
// because on this rig the suspects are all constraints: six position-driven arm
// motors with no torque limit, the ChLinkLockLock welding the arm to the chassis,
// and the weld that bonds a grabbed rock to the end effector.
void RobotRig::CheckDivergence(double time) {
    // Note the scan is NOT gated on the report count. Only PRINTING is capped.
    // Gating the scan would mean a rig that emitted its quota of warnings while
    // still finite could then go NaN without ever latching m_diverged -- i.e. the
    // detector would go quiet exactly as the failure completed, which is the same
    // fail-open bug this whole function exists to correct.
    if (m_diverged)
        return;
    if (time - m_last_divergence_scan < divergence_scan_period)
        return;
    m_last_divergence_scan = time;

    chrono::ChBody* nonfinite = nullptr;
    chrono::ChBody* fastest = nullptr;
    chrono::ChBody* spinniest = nullptr;
    double fastest_speed = 0.0;
    double spinniest_omega = 0.0;

    for (const auto& body : GetSystem()->GetBodies()) {
        const auto pos = body->GetPos();
        const auto rot = body->GetRot();
        const auto vel = body->GetPosDt();
        const auto omg = body->GetAngVelLocal();
        if (!IsFinite(pos) || !IsFinite(rot) || !IsFinite(vel) || !IsFinite(omg)) {
            if (!nonfinite)
                nonfinite = body.get();
            continue;  // magnitudes are meaningless once any component is NaN
        }
        const double speed = VecNorm(vel);
        if (speed > fastest_speed) {
            fastest_speed = speed;
            fastest = body.get();
        }
        const double omega = VecNorm(omg);
        if (omega > spinniest_omega) {
            spinniest_omega = omega;
            spinniest = body.get();
        }
    }

    const bool fatal = (nonfinite != nullptr);
    const bool warn = (fastest_speed > divergence_speed_warn) || (spinniest_omega > divergence_omega_warn);
    if (!fatal && !warn)
        return;

    // Latch first: a fatal finding must stop the job even if the print quota for
    // the warning tier is already spent.
    if (fatal)
        m_diverged = true;
    if (!fatal && m_divergence_reports >= divergence_max_reports)
        return;  // still scanning every cycle, just no longer shouting about it
    ++m_divergence_reports;

    const char* tag = fatal ? "DIVERGED" : "DIVERGING";
    std::cout << "[RobotRig] rank " << m_rank << " " << tag << " at t=" << time << "\n";

    auto report_body = [&](const char* what, chrono::ChBody* body, double magnitude) {
        if (!body)
            return;
        const auto p = body->GetPos();
        const auto v = body->GetPosDt();
        const auto w = body->GetAngVelLocal();
        // A negative magnitude means "not applicable" (the non-finite body, whose
        // numbers are meaningless). Printing a literal NaN here would be worse than
        // useless: the test harness greps the sim log for NaN as a failure signal,
        // so the detector would flag itself.
        std::cout << "[RobotRig] rank " << m_rank << " " << tag << "   " << what << " '" << body->GetName()
                  << "' magnitude=" << (magnitude < 0.0 ? std::string("n/a") : std::to_string(magnitude))
                  << " pos=(" << p.x() << ", " << p.y() << ", " << p.z()
                  << ") vel=(" << v.x() << ", " << v.y() << ", " << v.z() << ") omega=(" << w.x() << ", "
                  << w.y() << ", " << w.z() << ") fixed=" << (body->IsFixed() ? "yes" : "no")
                  << " collision=" << (body->IsCollisionEnabled() ? "on" : "off")
                  << " |Fcontact|=" << body->GetContactForce().Length() << " N\n";

        // Which constraints are pulling on it, and how hard. A position-driven
        // motor has no torque limit, so an impossible pose shows up here as a
        // reaction of absurd magnitude long before anything becomes NaN.
        for (const auto& link_base : GetSystem()->GetLinks()) {
            auto link = std::dynamic_pointer_cast<chrono::ChLink>(link_base);
            if (!link)
                continue;
            const auto* frame = static_cast<const chrono::ChBodyFrame*>(body);
            const bool first = (link->GetBody1() == frame);
            if (!first && link->GetBody2() != frame)
                continue;
            const auto wrench = first ? link->GetReaction1() : link->GetReaction2();
            std::cout << "[RobotRig] rank " << m_rank << " " << tag << "     link '" << link->GetName()
                      << "' |F|=" << wrench.force.Length() << " N |T|=" << wrench.torque.Length() << " Nm\n";
        }
    };

    if (fatal) {
        report_body("first non-finite body", nonfinite, -1.0);
        std::cout << "[RobotRig] rank " << m_rank << " " << tag
                  << "   this rank's physics is dead; every reading below it is meaningless\n";
    }
    if (fastest_speed > divergence_speed_warn || fatal)
        report_body("fastest body", fastest, fastest_speed);
    if (spinniest_omega > divergence_omega_warn || fatal)
        report_body("fastest-spinning body", spinniest, spinniest_omega);

    // What the rig was being asked to do. The arm is the prime suspect: its six
    // actuators are CONSTRAINT motors, so the solver must hit the commanded pose
    // with whatever reaction force that takes, and nothing upstream validates that
    // the pose is physically legal.
    if (m_arm) {
        const auto status = m_arm->GetStatus();
        std::cout << "[RobotRig] rank " << m_rank << " " << tag << "   arm phase=" << m_arm->GetPhaseName()
                  << " target=" << status.target_index << " state=" << status.state
                  << " err=" << status.error_code << " base_offset=" << m_arm->BaseOffsetFromChassis()
                  << " m (rigid mount, so a change here IS the arm tearing off the chassis)\n";
        const auto cmd = m_arm->GetCommandedTheta();
        const auto applied = m_arm->GetAppliedTheta();
        std::cout << "[RobotRig] rank " << m_rank << " " << tag << "   arm cmd_theta=(" << cmd[0] << ", "
                  << cmd[1] << ", " << cmd[2] << ", " << cmd[3] << ") applied=(" << applied[0] << ", "
                  << applied[1] << ", " << applied[2] << ", " << applied[3] << ")\n";
    }
    std::cout << "[RobotRig] rank " << m_rank << " " << tag << "   harvest_cycle=" << m_harvest_cycle
              << " dump_state=" << static_cast<int>(m_dump_state) << " rocks=" << m_rocks.size()
              << " raw(str/thr/brk)=" << m_last_raw_inputs.m_steering << "/" << m_last_raw_inputs.m_throttle
              << "/" << m_last_raw_inputs.m_braking << "\n";
    std::cout.flush();
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

    // NaN fails EVERY comparison, so without this line a diverged rank takes the
    // `!trying` branch below on every step and can never be reported stuck --
    // which is exactly how rank 4 disappeared for 90 minutes of a 3 h run with no
    // output at all. Divergence is CheckDivergence's to report; bail out here
    // rather than silently pretending the rig is fine.
    if (!std::isfinite(speed) || !std::isfinite(m_last_guarded_inputs.m_throttle))
        return;

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

    // WHAT IS TOUCHING THE GROUND that should not be.
    //
    // Every reading above can look normal on a rig that will not move: all four
    // wheels loaded, locked together, turning, ample engine torque, flat ground,
    // and still 0.0003 m/s. None of those say what is absorbing the thrust, so the
    // cause has been guessed at repeatedly instead of measured. This reports it
    // directly: any body in this rank's system carrying a contact force, other than
    // the wheels themselves (TMeasy tires are force elements with no collision
    // geometry, so a wheel spindle never shows up here anyway).
    //
    // A chassis, trailer frame, bed or arm link bearing load means the rig is
    // high-centred or dragging -- the tires deflect ~0.13 m under load, so the whole
    // vehicle rides that much lower than its design clearance.
    {
        int reported = 0;
        double total = 0.0;
        for (const auto& body : GetSystem()->GetBodies()) {
            const double f = body->GetContactForce().Length();
            if (f < 1.0)
                continue;
            total += f;
            if (reported++ < 8) {
                const auto bpos = body->GetPos();
                std::cout << "[RobotRig] rank " << m_rank << " STUCK   contact on '" << body->GetName()
                          << "' |F|=" << f << " N at (" << bpos.x() << ", " << bpos.y() << ", " << bpos.z()
                          << ")\n";
            }
        }
        std::cout << "[RobotRig] rank " << m_rank << " STUCK   " << reported
                  << " bodies in contact, total |F|=" << total << " N\n";
    }
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

void RobotRig::ApplySteeringStops(double time) {
    // WHAT THIS PREVENTS. Every double-wishbone upright in this vehicle is held against
    // rotating about its kingpin by ONE ChLinkDistance -- the tie rod, from the steering
    // link on the steered axle and from the chassis on the unsteered one. A distance
    // constraint between two points has two solutions, and for this suspension's
    // hardpoints the second one is a knuckle turned about -108 deg:
    //
    //   kingpin axis   UCA_U(-0.008, 0.571, 0.088) -> LCA_U(0.016, 0.574, -0.082)
    //   tie rod        0.44548 m, steering arm 0.1199 m about the kingpin
    //   |P_c - P_u| = 0.44548 is satisfied at theta = 0 AND at theta = -108 deg
    //
    // So a knuckle that gets pushed through its singular configuration -- where the tie
    // rod's moment arm about the kingpin passes through zero and the rod has no authority
    // at all -- lands on the far root and STAYS there. The constraint is satisfied. The
    // solver is not failing, there is nothing to converge, and no amount of iterations or
    // step reduction recovers it. Measured on run_20260825_221818: rank 2's right front
    // went from a steady +18.2 deg to +31.7 deg in one sample at t=118.0 and sat at
    // -117.1 deg for the remaining 482 s of the run, with the vehicle moving 52% of the
    // time; rank 1 did the same thing at t=110.0 and ended at -54.6 deg. Both robots.
    //
    // Widening the turns cut the trigger down to a single sample but cannot remove this,
    // because the far root is a legal state. The fix is to make it unreachable: a stiff
    // one-sided torque that switches on past stop_angle and never lets the knuckle get
    // near the singularity. Applied as a TORQUE, not a constraint, for the same reason
    // BuilderRig's park anchor is a force -- it ramps, the solver sees an ordinary
    // external load, and there is no discontinuity to tear the suspension apart.
    if (!m_vehicle)
        return;

    // stop_angle has to clear the largest angle the steering can legitimately command
    // and stay far below the far root. max_steering_angle_rad is 0.6 (34.4 deg), the
    // largest steady value logged is 19.4 deg, and the far root is at 108 deg -- so
    // 0.75 rad (43.0 deg) leaves 8.6 deg of headroom over full lock and 65 deg of
    // no-man's-land below the root.
    constexpr double stop_angle = 0.75;        // rad
    constexpr double stop_k = 1.5e4;           // N.m/rad
    constexpr double stop_c = 6.0e2;           // N.m.s/rad
    constexpr double stop_torque_max = 8.0e3;  // N.m
    // Past this, say so. A knuckle here is already through the singularity and the run
    // is producing garbage; the previous run spent 8.4 h of wall time in that state
    // because nothing was watching.
    constexpr double alarm_angle = 1.20;       // rad, 68.8 deg

    if (!m_steering_stops_ready) {
        m_steering_stops_ready = true;
        // The upright is not exposed by ChDoubleWishbone -- m_upright is protected and
        // there is no accessor -- but the SPINDLE is, and Chrono names the two from the
        // same stem. So take the spindle from the suspension, which is unambiguous, and
        // derive its upright's name from the actual spindle name rather than assuming
        // the "#2" suffix that duplicate names happen to get.
        for (int axle = 0; axle < static_cast<int>(m_vehicle->GetNumberAxles()); ++axle) {
            auto suspension = m_vehicle->GetSuspension(axle);
            if (!suspension)
                continue;
            for (auto side : {chrono::vehicle::LEFT, chrono::vehicle::RIGHT}) {
                auto spindle = suspension->GetSpindle(side);
                if (!spindle)
                    continue;
                const std::string tag = "_spindle";
                std::string name = spindle->GetName();
                const auto at = name.find(tag);
                if (at == std::string::npos)
                    continue;
                name.replace(at, tag.size(), "_upright");
                std::shared_ptr<chrono::ChBody> upright;
                for (const auto& body : GetSystem()->GetBodies()) {
                    if (body->GetName() == name) {
                        upright = body;
                        break;
                    }
                }
                if (!upright) {
                    std::cout << "[RobotRig] no upright named '" << name
                              << "'; no kingpin stop on that corner\n";
                    continue;
                }
                SteeringStop stop;
                stop.upright = upright;
                // Rest orientation in the CHASSIS frame, so static toe and camber cancel
                // and the measured angle is zero at the pose the vehicle was built in.
                stop.rest = m_vehicle->GetChassisBody()->GetRot().GetConjugate() * upright->GetRot();
                stop.accumulator = upright->AddAccumulator();
                // Axle AND side. The front knuckle is steered and the rear is not, so
                // "upright L" alone cannot say which of two different faults engaged.
                stop.label = (axle == 0) ? ((side == chrono::vehicle::LEFT) ? "front-L" : "front-R")
                                         : ((side == chrono::vehicle::LEFT) ? "rear-L" : "rear-R");
                m_steering_stops.push_back(stop);
            }
        }
        if (!m_steering_stops.empty())
            m_steering_stop_reaction = m_vehicle->GetChassisBody()->AddAccumulator();
        std::cout << "[RobotRig] kingpin stops on " << m_steering_stops.size()
                  << " uprights at +/-" << stop_angle << " rad\n";
    }

    const auto chassis = m_vehicle->GetChassisBody();
    const auto q_chassis = chassis->GetRot();
    // The kingpin is 8.1 deg off vertical, so the chassis z axis is the axis to measure
    // and push about: cos(8.1 deg) = 0.99 of the torque lands on the kingpin, and a
    // barrier does not need better than that.
    const auto axis_world = q_chassis.Rotate(chrono::ChVector3d(0.0, 0.0, 1.0));
    // Summed here and applied once, because all four stops react against the same body.
    double reaction = 0.0;

    for (auto& stop : m_steering_stops) {
        // Rotation of the upright away from its rest pose, expressed in the chassis
        // frame. Taking the z component of the rotation VECTOR rather than a Cardan
        // angle keeps it single-valued and continuous through the range that matters.
        const auto q_rel = q_chassis.GetConjugate() * stop.upright->GetRot();
        const auto q_delta = stop.rest.GetConjugate() * q_rel;
        const double theta = q_delta.GetRotVec().z();
        const double excess = std::abs(theta) - stop_angle;
        if (excess <= 0.0) {
            stop.upright->EmptyAccumulator(stop.accumulator);
            continue;
        }

        const auto w_rel = q_chassis.RotateBack(stop.upright->GetAngVelParent() -
                                               chassis->GetAngVelParent());
        double torque = -(theta > 0.0 ? 1.0 : -1.0) * stop_k * excess - stop_c * w_rel.z();
        torque = std::clamp(torque, -stop_torque_max, stop_torque_max);

        stop.upright->EmptyAccumulator(stop.accumulator);
        stop.upright->AccumulateTorque(stop.accumulator, torque * axis_world, false);
        // The reaction belongs on the chassis. Without it the stop injects angular
        // momentum into the vehicle every time it fires -- a torque out of nowhere,
        // applied precisely when the vehicle is already mishandling.
        reaction -= torque;

        if (!stop.engaged && m_steering_stop_reports < 20) {
            stop.engaged = true;
            ++m_steering_stop_reports;
            std::cout << "[RobotRig] kingpin stop engaged on upright " << stop.label
                      << " at t=" << time << " angle=" << theta << " rad ("
                      << theta * 180.0 / chrono::CH_PI << " deg)\n";
        }
        // Latched, like `engaged`. Unlatched, this is true on every step the knuckle
        // stays out there, so it would spend the whole 20-message budget in 20 steps --
        // 10 ms of sim -- and then go quiet for the rest of the run.
        if (std::abs(theta) > alarm_angle && !stop.alarmed) {
            stop.alarmed = true;
            std::cout << "[RobotRig] !! upright " << stop.label << " is at " << theta
                      << " rad (" << theta * 180.0 / chrono::CH_PI << " deg) at t=" << time
                      << " -- past the kingpin singularity; this corner is broken\n";
        }
    }

    if (!m_steering_stops.empty()) {
        chassis->EmptyAccumulator(m_steering_stop_reaction);
        if (reaction != 0.0)
            chassis->AccumulateTorque(m_steering_stop_reaction, reaction * axis_world, false);
    }
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
