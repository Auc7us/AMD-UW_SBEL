// Minimal MPI SynChrono demo for the AMD-UW Polaris JSON vehicle.
// Run with two ranks to see two synchronized Polaris vehicles on one flat terrain.

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <string>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChMassProperties.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/core/ChRotation.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMessageUtils.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::synchrono;
using namespace chrono::vehicle;

namespace {

// Simulation defaults shared by all MPI ranks.
const ChContactMethod contact_method = ChContactMethod::NSC;

double step_size = 3e-3;
double tire_step_size = 1e-3;
double end_time = 1000.0;
double heartbeat = 1e-2;
double render_step_size = 1.0 / 50.0;
double settle_time = 1.0;

const double terrain_resolution_scale = 3.0;
const double terrain_pixels_x = 133.0;
const double terrain_pixels_y = 33.0;
const double terrain_height_offset = 0.0;
const double terrain_min_height = 0.0;
const double terrain_max_height = 100.0;
const double terrain_height_probe_clearance = 10.0;
// Provisional lift of the chassis reference above the probed terrain surface.
// This only needs to be large enough that no wheel of the tractor or trailer
// starts buried in the terrain; the rig is then re-seated precisely (see
// seat_clearance) so the actual drop height no longer matters.
const double vehicle_start_clearance = 1.50;
// Final gap left between the LOWEST wheel of the whole rig (a trailer wheel)
// and the terrain after re-seating. Small enough that the touchdown is an
// effectively zero-velocity settle -- no impact, so even rigid tires on the
// light trailer don't bounce.
const double seat_clearance = 0.025;

ChVector3d track_point(0.0, 0.0, 1.0);

// Small adapter so the simulation loop can stay headless unless this rank owns a VSG window.
class VsgAppWrapper {
  public:
    void Set(std::shared_ptr<ChWheeledVehicleVisualSystemVSG> app) { m_app = app; }

    bool IsOk() const { return m_app ? m_app->Run() : true; }

    void Synchronize(double time, const DriverInputs& driver_inputs) {
        if (m_app)
            m_app->Synchronize(time, driver_inputs);
    }

    void Advance(double step) {
        if (m_app)
            m_app->Advance(step);
    }

    void Render() {
        if (!m_app)
            return;

        m_app->Render();
    }

  private:
    std::shared_ptr<ChWheeledVehicleVisualSystemVSG> m_app;
};

// Driver adapter that starts with a full brake hold. The hold is released once
// the interactive driver receives a real throttle or steering command.
class DriverWrapper : public ChDriver {
  public:
    explicit DriverWrapper(ChVehicle& vehicle) : ChDriver(vehicle), m_hold_brake(true) {
        m_throttle = 0.0;
        m_steering = 0.0;
        m_braking = 1.0;
    }

    void Set(std::shared_ptr<ChInteractiveDriver> driver) { m_driver = driver; }

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
    std::shared_ptr<ChInteractiveDriver> m_driver;
    bool m_hold_brake;
};

// SynChrono agent that broadcasts a WheeledTrailer so it shows up on remote
// ranks. A WheeledTrailer is not a ChWheeledVehicle, so it cannot be handed to
// a stock SynWheeledVehicleAgent. Instead we reuse that agent's zombie
// machinery (a chassis mesh + N wheel meshes driven by a state message) and
// override only how the *local* state is sampled, reading the trailer's
// chassis body and spindles exactly the way the base class reads a vehicle's.
class SynTrailerAgent : public SynWheeledVehicleAgent {
  public:
    explicit SynTrailerAgent(std::shared_ptr<WheeledTrailer> trailer = nullptr)
        : SynWheeledVehicleAgent(nullptr), m_trailer(trailer) {}

    void Update() override {
        if (!m_trailer)
            return;  // zombie instance on a remote rank: nothing local to sample

        auto chassis_abs = m_trailer->GetChassis()->GetBody()->GetFrameRefToAbs();
        SynPose chassis(chassis_abs.GetPos(), chassis_abs.GetRot());
        chassis.GetFrame().SetPosDt(chassis_abs.GetPosDt());
        chassis.GetFrame().SetPosDt2(chassis_abs.GetPosDt2());
        chassis.GetFrame().SetRotDt(chassis_abs.GetRotDt());
        chassis.GetFrame().SetRotDt2(chassis_abs.GetRotDt2());

        std::vector<SynPose> wheels;
        for (auto& axle : m_trailer->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto state = wheel->GetState();
                auto wheel_abs = wheel->GetSpindle()->GetFrameRefToAbs();
                SynPose frame(state.pos, state.rot);
                frame.GetFrame().SetPosDt(wheel_abs.GetPosDt());
                frame.GetFrame().SetPosDt2(wheel_abs.GetPosDt2());
                frame.GetFrame().SetRotDt(wheel_abs.GetRotDt());
                frame.GetFrame().SetRotDt2(wheel_abs.GetRotDt2());
                wheels.emplace_back(frame);
            }
        }

        const double time = m_trailer->GetChassis()->GetBody()->GetSystem()->GetChTime();
        m_state->SetState(time, chassis, wheels);
    }

  private:
    std::shared_ptr<WheeledTrailer> m_trailer;
};

// Command-line options used by the demo.
void AddCommandLineOptions(ChCLI& cli) {
    cli.AddOption<double>("Simulation", "s,step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "e,end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "b,heartbeat", "SynChrono heartbeat", std::to_string(heartbeat));
    cli.AddOption<double>("Simulation", "settle_time", "Pre-run time with full brake before Synchrono starts",
                          std::to_string(settle_time));
    cli.AddOption<std::vector<int>>("VSG", "vsg", "MPI ranks that should open VSG visualization", "-1");
}

}  // namespace

int main(int argc, char* argv[]) {
    // MPI/SynChrono setup. Each MPI rank owns one real Polaris vehicle.
    auto communicator = chrono_types::make_shared<SynMPICommunicator>(argc, argv);
    const int rank = communicator->GetRank();
    const int num_ranks = communicator->GetNumRanks();

    SynChronoManager syn_manager(rank, num_ranks, communicator);

    ChCLI cli(argv[0]);
    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, rank == 0))
        return 0;

    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<double>("heartbeat");
    settle_time = cli.GetAsType<double>("settle_time");
    syn_manager.SetHeartbeat(heartbeat);

    // Use AMD-UW data as the Chrono data root and its vehicle subfolder for vehicle JSON assets.
    std::string amd_uw_data_path = UW_AMD_DATA_DIR;
    if (!amd_uw_data_path.empty() && amd_uw_data_path.back() != '/')
        amd_uw_data_path += "/";

    std::string vehicle_data_path = UW_AMD_VEHICLE_DATA_DIR;
    if (!vehicle_data_path.empty() && vehicle_data_path.back() != '/')
        vehicle_data_path += "/";
    SetChronoDataPath(vehicle_data_path);
    SetVehicleDataPath(vehicle_data_path);

    if (rank == 0) {
        SynLog() << "Chrono version: " << CHRONO_VERSION << "\n";
        SynLog() << "MPI ranks: " << num_ranks << "\n";
        SynLog() << "Vehicle data: " << GetVehicleDataPath() << "\n\n";
    }

    const double terrain_length = terrain_pixels_x * terrain_resolution_scale;
    const double terrain_width = terrain_pixels_y * terrain_resolution_scale;

    // Local dynamic vehicle for this rank.
    WheeledVehicle vehicle(GetVehicleDataFile("LRV/Polaris.json"), contact_method);
    vehicle.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Apollo-site height-map terrain with Bekker-Wong SCM soil contact model.
    SCMTerrain terrain(vehicle.GetSystem());
    terrain.SetSoilParameters(
        2.8e6,    // Bekker Kphi, frictional modulus [Pa/m^n]
        14e3,     // Bekker Kc, cohesive modulus [Pa/m^(n-1)]
        1.0,      // Bekker n, sinkage exponent
        1000.0,   // Mohr cohesion c [Pa]
        35.0,     // Mohr friction phi [deg]
        0.02,     // Janosi-Hanamoto shear parameter K [m]
        2e8,      // elastic stiffness [Pa/m] (must exceed Kphi)
        3e4       // damping R [Pa.s/m]
    );
    terrain.SetPlotType(SCMTerrain::PLOT_SINKAGE, 0, 0.1);
    terrain.Initialize(amd_uw_data_path + "terrain/nasa_apollo_site.bmp",
                       terrain_length, terrain_width, terrain_min_height, terrain_max_height,
                       0.1);   // SCM grid resolution [m] — must be ≤ half the tire width (0.114 m)

    // SCMTerrain::GetHeight() queries the terrain grid directly (no ray cast),
    // so it is valid immediately after Initialize() — no BindAll() needed here.

    // Scatter rocks across the terrain. Added before the preexisting_bodies
    // snapshot so the re-seating step below correctly skips them (rocks are
    // positioned relative to the terrain, not to the vehicle chassis).
    // XY positions spread around the spawn origin; Z is sampled from terrain
    // and the rock is lifted slightly (rock_clearance) so it rests on top.
    std::vector<std::shared_ptr<ChBody>> rock_bodies;
    {
        const std::vector<ChVector3d> rock_xy = {
            { 5.0, -1.5, 0}, { 8.0,  1.0, 0}, {12.0,  0.5, 0}, {15.0, -2.0, 0}, {20.0,  1.5, 0},
            {-3.0,  2.0, 0}, {-6.0, -1.0, 0}, { 3.0,  3.0, 0}, {10.0,  2.5, 0}, {18.0, -1.0, 0},
            { 7.0, -3.0, 0}, {13.0,  3.0, 0}, {-4.0, -3.0, 0}, { 2.0, -2.5, 0}, {22.0,  0.0, 0},
            { 9.0,  0.0, 0}, {16.0,  2.0, 0}, {-8.0,  1.5, 0}, { 6.0,  2.0, 0}, {11.0, -1.5, 0},
        };

        // Scale and rotation seeds — vary per rock for visual variety.
        const double rock_scales[] = {0.20, 0.13, 0.18, 0.10, 0.22,
                                      0.15, 0.25, 0.12, 0.17, 0.21,
                                      0.14, 0.19, 0.11, 0.23, 0.16,
                                      0.24, 0.13, 0.20, 0.15, 0.18};
        // Z-rotation angles (radians) to avoid all rocks looking identical.
        const double rock_angles_z[] = {0.0,  0.8,  1.6,  2.4,  3.1,
                                        0.4,  1.2,  2.0,  2.8,  0.6,
                                        1.4,  2.2,  3.0,  0.2,  1.0,
                                        1.8,  2.6,  0.9,  1.7,  2.5};
        const double rock_clearance = 0.05;
        const std::string rock_mesh_path = GetVehicleDataFile("terrain/obstacles/rock.obj");

        auto rock_surf_mat = ChContactMaterial::DefaultMaterial(contact_method);
        rock_surf_mat->SetFriction(0.8f);
        rock_surf_mat->SetRestitution(0.05f);

        auto rock_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        rock_vis_mat->SetAmbientColor({1, 1, 1});
        rock_vis_mat->SetDiffuseColor({1, 1, 1});
        rock_vis_mat->SetSpecularColor({1, 1, 1});
        rock_vis_mat->SetUseSpecularWorkflow(true);
        rock_vis_mat->SetRoughness(1.0f);
        rock_vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f,
                                         23.4f * (CH_PI / 180));

        for (int i = 0; i < static_cast<int>(rock_xy.size()); i++) {
            auto rock_mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_mesh_path, false, true);
            rock_mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(rock_scales[i]));
            rock_mesh->RepairDuplicateVertices(1e-9);

            double mmass;
            ChVector3d mcog;
            ChMatrix33<> minertia;
            const double rock_density = 2500.0;
            rock_mesh->ComputeMassProperties(true, mmass, mcog, minertia);
            ChMatrix33<> principal_rot;
            ChVector3d principal_I;
            ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_rot);

            const double rock_probe_z = terrain_height_offset + terrain_max_height + terrain_height_probe_clearance;
            const double surface_z = terrain.GetHeight(ChVector3d(rock_xy[i].x(), rock_xy[i].y(), rock_probe_z));
            const ChVector3d rock_pos(rock_xy[i].x(), rock_xy[i].y(), surface_z + rock_clearance);
            const ChQuaternion<> rock_rot = QuatFromAngleX(CH_PI / 2) * QuatFromAngleZ(rock_angles_z[i]);

            auto rock_body = chrono_types::make_shared<ChBodyAuxRef>();
            rock_body->SetFrameCOMToRef(ChFrame<>(mcog, principal_rot));
            rock_body->SetMass(mmass * rock_density);
            rock_body->SetInertiaXX(rock_density * principal_I);
            rock_body->SetFrameRefToAbs(ChFrame<>(rock_pos, rock_rot));
            rock_body->SetFixed(false);

            auto rock_col = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                rock_surf_mat, rock_mesh, false, false, 0.005);
            rock_body->AddCollisionShape(rock_col);
            rock_body->EnableCollision(true);

            auto rock_vis_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            rock_vis_mesh->SetMesh(rock_mesh);
            rock_vis_mesh->SetBackfaceCull(true);
            if (rock_vis_mesh->GetNumMaterials() == 0)
                rock_vis_mesh->AddMaterial(rock_vis_mat);
            else
                rock_vis_mesh->GetMaterials()[0] = rock_vis_mat;
            rock_body->AddVisualShape(rock_vis_mesh);

            vehicle.GetSystem()->Add(rock_body);
            rock_bodies.push_back(rock_body);
        }

        if (rank == 0)
            SynLog() << "Placed " << rock_xy.size() << " rocks on terrain.\n";
    }

    const double start_x = 0.0;
    const double start_y = rank * 4.0;
    // Probe from above the tallest possible terrain so the downward ray cast hits.
    const double height_probe_z = terrain_height_offset + terrain_max_height + terrain_height_probe_clearance;
    const double start_z = terrain.GetHeight(ChVector3d(start_x, start_y, height_probe_z)) + vehicle_start_clearance;
    const ChVector3d init_loc(start_x, start_y, start_z);
    const ChQuaternion<> init_rot(1, 0, 0, 0);

    // Snapshot the bodies that exist before the rig is built (the terrain patch
    // and anything else). These must stay put when we re-seat the rig below.
    std::set<ChBody*> preexisting_bodies;
    for (const auto& body : vehicle.GetSystem()->GetBodies())
        preexisting_bodies.insert(body.get());

    vehicle.Initialize(ChCoordsys<>(init_loc, init_rot));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    auto engine = ReadEngineJSON(GetVehicleDataFile("LRV/Polaris_EngineSimpleMap.json"));
    auto transmission = ReadTransmissionJSON(GetVehicleDataFile("LRV/Polaris_AutomaticTransmissionSimpleMap.json"));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(GetVehicleDataFile("LRV/Polaris_RigidTire.json"));
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
            tire->SetStepsize(tire_step_size);
        }
    }

    // Local trailer attached to this rank's Polaris chassis.
    auto trailer = chrono_types::make_shared<WheeledTrailer>(vehicle.GetSystem(),
                                                             GetVehicleDataFile("LRV_Wagon/Polaris.json"));
    trailer->Initialize(vehicle.GetChassis());
    trailer->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    trailer->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    trailer->SetWheelVisualizationType(VisualizationType::PRIMITIVES);

    for (auto& axle : trailer->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            // Use the same Polaris tire as the tractor (radius 0.4089). The
            // utility-trailer tire (LRV_Wagon/UT_RigidTire.json, radius 0.16) is
            // far too small for the Polaris ride height, leaving the trailer
            // wheels floating above the ground so the trailer swings/bounces.
            auto tire = ReadTireJSON(GetVehicleDataFile("LRV/Polaris_RigidTire.json"));
            trailer->InitializeTire(tire, wheel, VisualizationType::MESH);
            tire->SetStepsize(tire_step_size);
        }
    }

    // Register active domains with SCM so it computes Bekker-Wong forces on each
    // wheel and rock. Without these, SCM has no contact geometry to push against
    // and everything sinks through the deformable mesh.
    {
        const double wr = 0.55;  // half-extent slightly larger than Polaris tire radius (0.41 m)
        for (auto& axle : vehicle.GetAxles())
            for (auto& wheel : axle->GetWheels())
                terrain.AddActiveDomain(wheel->GetSpindle(), ChVector3d(0, 0, 0),
                                        ChVector3d(wr, 2 * wr, 2 * wr));
        for (auto& axle : trailer->GetAxles())
            for (auto& wheel : axle->GetWheels())
                terrain.AddActiveDomain(wheel->GetSpindle(), ChVector3d(0, 0, 0),
                                        ChVector3d(wr, 2 * wr, 2 * wr));
        for (auto& rock : rock_bodies)
            terrain.AddActiveDomain(rock, ChVector3d(0, 0, 0), ChVector3d(0.4, 0.4, 0.4));
    }

    // Re-seat the whole rig on the terrain. The provisional pose above dropped
    // everything from vehicle_start_clearance, which is fragile: too small and
    // the (lower-riding) trailer wheels start buried and tunnel through; too
    // large and the light trailer hits hard and bounces on its rigid tires.
    // Instead, find the lowest wheel contact point across BOTH tractor and
    // trailer and translate the entire assembly down as one rigid body so that
    // lowest wheel rests seat_clearance above the surface. A uniform z-shift
    // preserves every relative joint/spring frame (including the hitch) and
    // leaves all velocities at zero, so there is no constraint violation and
    // the touchdown carries essentially no impact energy.
    {
        double lowest_bottom = std::numeric_limits<double>::infinity();
        double lowest_x = start_x, lowest_y = start_y;
        auto consider_wheel = [&](const auto& wheel) {
            if (!wheel)
                return;
            const auto& tire = wheel->GetTire();
            const double radius = tire ? tire->GetRadius() : 0.0;
            const ChVector3d p = wheel->GetPos();
            const double bottom = p.z() - radius;
            if (bottom < lowest_bottom) {
                lowest_bottom = bottom;
                lowest_x = p.x();
                lowest_y = p.y();
            }
        };
        for (auto& axle : vehicle.GetAxles())
            for (auto& wheel : axle->GetWheels())
                consider_wheel(wheel);
        for (auto& axle : trailer->GetAxles())
            for (auto& wheel : axle->GetWheels())
                consider_wheel(wheel);

        const double terrain_under_lowest =
            terrain.GetHeight(ChVector3d(lowest_x, lowest_y, height_probe_z));
        const double drop = lowest_bottom - (terrain_under_lowest + seat_clearance);

        for (const auto& body : vehicle.GetSystem()->GetBodies()) {
            if (preexisting_bodies.count(body.get()))
                continue;  // terrain etc.: leave in place
            const ChVector3d p = body->GetPos();
            body->SetPos(ChVector3d(p.x(), p.y(), p.z() - drop));
        }

        if (rank == 0) {
            SynLog() << "Re-seated rig: lowered by " << drop << " m so lowest wheel sits "
                     << seat_clearance << " m above terrain.\n";
        }
    }

    // The trailer chassis JSON defines only visualization primitives and has no
    // collision geometry, so cargo would fall through the bed. Add a thin,
    // collidable slab pinned to the trailer chassis to act as that bed surface.
    // It is kept fixed (no dynamics) and repositioned to follow the chassis each
    // step in the simulation loop below.
    auto trailer_bed_mat = ChContactMaterial::DefaultMaterial(contact_method);
    trailer_bed_mat->SetFriction(0.9f);
    auto trailer_bed = chrono_types::make_shared<ChBodyEasyBox>(1.0, 1.2, 0.02, 1000.0,
                                                                /*visualize=*/false,
                                                                /*collide=*/true, trailer_bed_mat);
    trailer_bed->SetFixed(true);
    trailer_bed->EnableCollision(true);
    trailer_bed->SetPos(trailer->GetChassis()->GetPos() + ChVector3d(0, 0, 0.01));
    trailer_bed->SetRot(trailer->GetChassis()->GetRot());
    vehicle.GetSystem()->AddBody(trailer_bed);
    DriverWrapper driver(vehicle);
    VsgAppWrapper app;

    // Interactive keyboard driver. The wrapper above keeps the vehicle braked until input arrives.
    auto irr_driver = chrono_types::make_shared<ChInteractiveDriver>(vehicle);
    irr_driver->SetSteeringDelta(render_step_size / 1.0);
    irr_driver->SetThrottleDelta(render_step_size / 1.0);
    irr_driver->SetBrakingDelta(render_step_size / 0.3);
    irr_driver->Initialize();
    driver.Set(irr_driver);

    // Register ALL collision shapes (spindles, rocks, trailer bed, etc.) with Bullet now
    // that every body has been added to the system. SCM's ray casting needs to find the
    // tire cylinders; SyncCollisionModels() only moves already-bound shapes, so bodies
    // added since the last BindAll() are invisible to ray tests until this call.
    vehicle.GetSystem()->GetCollisionSystem()->BindAll();

    // Optional pre-run settling under full brake before Synchrono state exchange begins.
    if (settle_time > 0) {
        DriverInputs brake_inputs = driver.GetInputs();
        const int settle_steps = static_cast<int>(std::ceil(settle_time / step_size));

        for (int i = 0; i < settle_steps; i++) {
            const double time = vehicle.GetSystem()->GetChTime();
            terrain.Synchronize(time);
            vehicle.Synchronize(time, brake_inputs, terrain);
            trailer->Synchronize(time, brake_inputs, terrain);
            terrain.Advance(step_size);
            vehicle.Advance(step_size);
            trailer->Advance(step_size);
        }

        vehicle.GetSystem()->SetChTime(0.0);
    }

    // Synchrono agent for this vehicle. Relative mesh paths are supplied explicitly for remote zombie visualization.
    auto vehicle_agent = chrono_types::make_shared<SynWheeledVehicleAgent>(&vehicle);
    vehicle_agent->SetZombieVisualizationFiles("LRV/meshes/Polaris_chassis.obj",
                                               "LRV/meshes/Polaris_wheel.obj",
                                               "LRV/meshes/Polaris_tire.obj");
    vehicle_agent->SetNumWheels(4);
    syn_manager.AddAgent(vehicle_agent);

    // Synchrono agent for this vehicle's trailer, so the other rank's trailer is
    // drawn as a zombie too (not just the tractor). The trailer chassis JSON
    // only defines primitives, so we point the zombie at the trailer mesh and
    // reuse the Polaris wheel/tire meshes (the trailer rolls on Polaris tires).
    // NOTE: every rank must add agents in the same order -- vehicle then trailer
    // -- so the (node, agent) keys line up across ranks for zombie matching.
    auto trailer_agent = chrono_types::make_shared<SynTrailerAgent>(trailer);
    trailer_agent->SetZombieVisualizationFiles("LRV_Wagon/trailer_chassis.obj",
                                               "LRV/meshes/Polaris_wheel.obj",
                                               "LRV/meshes/Polaris_tire.obj");
    trailer_agent->SetNumWheels(2);
    syn_manager.AddAgent(trailer_agent);

    syn_manager.Initialize(vehicle.GetSystem());

    // Optional VSG visualization for selected ranks.
    if (cli.HasValueInVector<int>("vsg", rank)) {
        SetChronoDataPath(amd_uw_data_path);
        auto vsg_app = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
        vsg_app->SetWindowTitle("AMD-UW SynChrono Polaris Apollo Terrain Demo");
        vsg_app->SetWindowSize(1280, 800);
        vsg_app->SetWindowPosition(100, 100);
        vsg_app->SetChaseCamera(track_point, 8.0, 0.75);
        vsg_app->SetChaseCameraPosition(ChVector3d(-7.0, 4.0, 2.0));
        vsg_app->SetSkyDomeTexture(GetChronoDataFile("skybox/lunar_stars_dome.png"), CH_PI);
        vsg_app->SetLightIntensity(1.0f);
        vsg_app->SetLightDirection(CH_PI, 1.37);
        vsg_app->EnableSkyTexture(SkyMode::DOME);
        vsg_app->EnableShadows();
        vsg_app->AttachVehicle(&vehicle);
        vsg_app->AttachDriver(irr_driver.get());
        vsg_app->AttachTerrain(&terrain);
        vsg_app->Initialize();
        app.Set(vsg_app);
    }

    const int render_steps = static_cast<int>(std::ceil(render_step_size / step_size));
    int step_number = 0;
    ChRealtimeStepTimer realtime_timer;
    const auto wall_start = std::chrono::high_resolution_clock::now();

    // Main simulation loop: exchange Synchrono state, update vehicle/terrain/driver, and render if enabled.
    while (app.IsOk() && syn_manager.IsOk()) {
        const double time = vehicle.GetSystem()->GetChTime();
        if (time >= end_time)
            break;

        if (step_number % render_steps == 0)
            app.Render();

        syn_manager.Synchronize(time);
        driver.Synchronize(time);
        const DriverInputs driver_inputs = driver.GetInputs();
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        trailer->Synchronize(time, driver_inputs, terrain);
        app.Synchronize(time, driver_inputs);

        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        trailer->Advance(step_size);
        app.Advance(step_size);

        // Keep the (fixed) cargo-bed slab glued to the trailer chassis.
        trailer_bed->SetPos(trailer->GetChassis()->GetPos() + ChVector3d(0, 0, 0.01));
        trailer_bed->SetRot(trailer->GetChassis()->GetRot());

        if (rank == 0 && step_number % 1000 == 0 && step_number > 0) {
            const auto wall_now = std::chrono::high_resolution_clock::now();
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(wall_now - wall_start);
            SynLog() << "time=" << time << "  wall/sim=" << (elapsed.count() / 1000.0) / time << "\n";
        }

        realtime_timer.Spin(step_size);
        step_number++;
    }

    syn_manager.QuitSimulation();
    return 0;
}
