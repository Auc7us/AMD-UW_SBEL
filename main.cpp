// Minimal MPI SynChrono demo for the AMD-UW Polaris JSON vehicle.
// Run with two ranks to see two synchronized Polaris vehicles on one flat terrain.

#include <chrono>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/core/ChTypes.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChMassProperties.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/flatbuffer/message/SynApproachMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMessageUtils.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "src/MaterialUtils.h"
#include "src/RockField.h"
#include "src/SynAgents.h"

using namespace chrono;
using namespace chrono::sensor;
using namespace chrono::synchrono;
using namespace chrono::vehicle;
using namespace amd_uw;

namespace {

// Simulation defaults shared by all MPI ranks.
const ChContactMethod contact_method = ChContactMethod::NSC;

double step_size = 2e-3;
double tire_step_size = 1e-3;
double end_time = 1000.0;
double heartbeat = 1e-2;
double render_step_size = 1.0 / 50.0;
double settle_time = 1.0;
double motion_log_rate = 0.0;

const double terrain_resolution_scale = 4.0;
const double terrain_pixels_x = 256.0;
const double terrain_pixels_y = 256.0;
const std::string terrain_heightmap_file = "terrain/terrain2.bmp";
const double terrain_height_offset = 0.0;
const double terrain_min_height = -25.0;
const double terrain_max_height = 25.0;
const double terrain_height_probe_clearance = 10.0;
const RockFieldConfig rock_field_config;
const float global_camera_update_rate = 30.0f;
const unsigned int global_camera_width = 1280;
const unsigned int global_camera_height = 720;
const float global_camera_fov = static_cast<float>(CH_PI_3);
const ChVector3d global_camera_position(-100.0, 30.0, 30.0);
const ChVector3d global_camera_target(30.0, 0.0, 5.0);
const char* sensor_star_map = "sensor/textures/starmap_2020_4k.hdr";
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

double VecNorm(const ChVector3d& v) {
    return std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
}

ChQuaternion<> SensorLookAtRotation(const ChVector3d& camera_pos, const ChVector3d& target_pos) {
    const ChVector3d forward = (target_pos - camera_pos).GetNormalized();
    ChMatrix33<> rot;
    rot.SetFromAxisX(forward, VECT_Y);
    return rot.GetQuaternion();
}

double InitialHeadingDegForRobot(int robot_index) {
    return (robot_index == 0) ? 330.0 : 60.0;
}

ChVector3d InitialGroundPositionForRobot(int robot_index, int num_robots, double start_spacing) {
    return ChVector3d(0.0, (robot_index - 0.5 * (num_robots - 1)) * start_spacing, 0.0);
}

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

// Command-line options used by the demo.
void AddCommandLineOptions(ChCLI& cli) {
    cli.AddOption<double>("Simulation", "s,step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "e,end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "b,heartbeat", "SynChrono heartbeat", std::to_string(heartbeat));
    cli.AddOption<double>("Simulation", "settle_time", "Pre-run time with full brake before Synchrono starts",
                          std::to_string(settle_time));
    cli.AddOption<double>("Diagnostics", "motion_log_rate",
                          "Rank-local chassis and tire-force log rate in Hz (0 disables logging)",
                          std::to_string(motion_log_rate));
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
    motion_log_rate = cli.GetAsType<double>("motion_log_rate");
    syn_manager.SetHeartbeat(heartbeat);

    // Use AMD-UW data as the Chrono data root and its vehicle subfolder for vehicle JSON assets.
    std::string amd_uw_data_path = UW_AMD_DATA_DIR;
    if (!amd_uw_data_path.empty() && amd_uw_data_path.back() != '/')
        amd_uw_data_path += "/";

    std::string chrono_data_path = CHRONO_DATA_DIR;
    if (!chrono_data_path.empty() && chrono_data_path.back() != '/')
        chrono_data_path += "/";

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

    const bool is_sensor_rank = (rank == 0);
    const bool owns_robot = (rank > 0);
    const int num_robot_ranks = std::max(0, num_ranks - 1);

    ChSystemNSC sensor_system;
    sensor_system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    std::unique_ptr<WheeledVehicle> vehicle;
    ChSystem* system = &sensor_system;
    if (owns_robot) {
        vehicle = std::make_unique<WheeledVehicle>(GetVehicleDataFile("LRV/Polaris.json"), contact_method);
        system = vehicle->GetSystem();
        system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    }

    // Apollo-site height-map terrain. Each BMP pixel maps to an integer-sized terrain cell.
    RigidTerrain terrain(system);
    auto ground_mat = ChContactMaterial::DefaultMaterial(contact_method);
    ground_mat->SetFriction(0.9f);
    ground_mat->SetRestitution(0.0f);
    const ChCoordsys<> terrain_csys(ChVector3d(0.0, 0.0, terrain_height_offset), QUNIT);
    auto ground = terrain.AddPatch(ground_mat, terrain_csys, amd_uw_data_path + terrain_heightmap_file,
                                   terrain_length, terrain_width, terrain_min_height, terrain_max_height);
    ground->SetColor(ChColor(0.55f, 0.55f, 0.52f));
    terrain.Initialize();
    ApplyMaterialToVisualShapes(ground->GetGroundBody(), CreateLunarHapkeMaterial());

    // RigidTerrain::GetHeight() works by casting a vertical ray into the patch
    // collision model. That model is only inserted into the Bullet collision
    // world when the system is initialized on the first DoStepDynamics(), so a
    // probe issued here (before any step) would always miss and return 0 -- which
    // is the bug that forced the large manual spawn offset. Bind the collision
    // models now so the height probe below hits the real terrain surface. The
    // call is idempotent: already-bound models are skipped, and the vehicle/
    // trailer models added later are still bound on the first step.
    system->GetCollisionSystem()->BindAll();

    const double start_spacing = 50.0;
    // Probe from above the tallest possible terrain so the downward ray cast hits.
    const double height_probe_z = terrain_height_offset + terrain_max_height + terrain_height_probe_clearance;
    auto rock_mat = ChContactMaterial::DefaultMaterial(contact_method);
    rock_mat->SetFriction(0.9f);
    rock_mat->SetRestitution(0.0f);
    std::vector<std::shared_ptr<ChBodyAuxRef>> owned_rocks;
    std::shared_ptr<WheeledTrailer> trailer;
    std::shared_ptr<ChBodyEasyBox> trailer_bed;
    std::unique_ptr<DriverWrapper> driver;
    std::shared_ptr<ChInteractiveDriver> irr_driver;
    VsgAppWrapper app;

    if (owns_robot) {
        const int robot_index = rank - 1;
        owned_rocks = AddRockFields(system, terrain, rock_mat, chrono_data_path, amd_uw_data_path, robot_index,
                                    num_robot_ranks, start_spacing, height_probe_z, rock_field_config);

        const ChVector3d start_ground = InitialGroundPositionForRobot(robot_index, num_robot_ranks, start_spacing);
        const double start_x = start_ground.x();
        const double start_y = start_ground.y();
        const double start_z =
            terrain.GetHeight(ChVector3d(start_x, start_y, height_probe_z)) + vehicle_start_clearance;
        const ChVector3d init_loc(start_x, start_y, start_z);
        const double init_heading_deg = InitialHeadingDegForRobot(robot_index);
        const ChQuaternion<> init_rot = QuatFromAngleZ(init_heading_deg * CH_DEG_TO_RAD);

        std::set<ChBody*> preexisting_bodies;
        for (const auto& body : system->GetBodies())
            preexisting_bodies.insert(body.get());

        vehicle->Initialize(ChCoordsys<>(init_loc, init_rot));
        vehicle->GetChassis()->SetFixed(false);
        vehicle->SetChassisVisualizationType(VisualizationType::MESH);
        vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
        vehicle->SetWheelVisualizationType(VisualizationType::MESH);

        auto engine = ReadEngineJSON(GetVehicleDataFile("LRV/Polaris_EngineSimpleMap.json"));
        auto transmission = ReadTransmissionJSON(GetVehicleDataFile("LRV/Polaris_AutomaticTransmissionSimpleMap.json"));
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        vehicle->InitializePowertrain(powertrain);

        for (auto& axle : vehicle->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile("LRV/Polaris_RigidTire.json"));
                vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
                tire->SetStepsize(tire_step_size);
            }
        }

        trailer = chrono_types::make_shared<WheeledTrailer>(system, GetVehicleDataFile("LRV_Wagon/Polaris.json"));
        trailer->Initialize(vehicle->GetChassis());
        trailer->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
        trailer->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        trailer->SetWheelVisualizationType(VisualizationType::PRIMITIVES);

        for (auto& axle : trailer->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile("LRV/Polaris_RigidTire.json"));
                trailer->InitializeTire(tire, wheel, VisualizationType::MESH);
                tire->SetStepsize(tire_step_size);
            }
        }

        {
            double min_clearance = std::numeric_limits<double>::infinity();
            auto consider_wheel = [&](const auto& wheel) {
                if (!wheel)
                    return;
                const auto& tire = wheel->GetTire();
                const double radius = tire ? tire->GetRadius() : 0.0;
                const ChVector3d p = wheel->GetPos();
                const double bottom = p.z() - radius;
                const double terrain_under_wheel = terrain.GetHeight(ChVector3d(p.x(), p.y(), height_probe_z));
                min_clearance = std::min(min_clearance, bottom - terrain_under_wheel);
            };
            for (auto& axle : vehicle->GetAxles())
                for (auto& wheel : axle->GetWheels())
                    consider_wheel(wheel);
            for (auto& axle : trailer->GetAxles())
                for (auto& wheel : axle->GetWheels())
                    consider_wheel(wheel);

            const double drop = min_clearance - seat_clearance;

            for (const auto& body : system->GetBodies()) {
                if (preexisting_bodies.count(body.get()))
                    continue;
                const ChVector3d p = body->GetPos();
                body->SetPos(ChVector3d(p.x(), p.y(), p.z() - drop));
            }

            SynLog() << "Re-seated rank " << rank << " rig: lowered by " << drop << " m.\n";
        }

        auto trailer_bed_mat = ChContactMaterial::DefaultMaterial(contact_method);
        trailer_bed_mat->SetFriction(0.9f);
        trailer_bed = chrono_types::make_shared<ChBodyEasyBox>(1.0, 1.2, 0.02, 1000.0,
                                                               /*visualize=*/false,
                                                               /*collide=*/true, trailer_bed_mat);
        trailer_bed->SetFixed(true);
        trailer_bed->EnableCollision(true);
        trailer_bed->SetPos(trailer->GetChassis()->GetPos() + ChVector3d(0, 0, 0.01));
        trailer_bed->SetRot(trailer->GetChassis()->GetRot());
        system->AddBody(trailer_bed);

        driver = std::make_unique<DriverWrapper>(*vehicle);
        irr_driver = chrono_types::make_shared<ChInteractiveDriver>(*vehicle);
        irr_driver->SetSteeringDelta(render_step_size / 1.0);
        irr_driver->SetThrottleDelta(render_step_size / 1.0);
        irr_driver->SetBrakingDelta(render_step_size / 0.3);
        irr_driver->Initialize();
        driver->Set(irr_driver);

        if (settle_time > 0) {
            DriverInputs brake_inputs = {0.0, 0.0, 1.0, 0.0};
            const int settle_steps = static_cast<int>(std::ceil(settle_time / step_size));

            for (int i = 0; i < settle_steps; i++) {
                const double time = system->GetChTime();
                terrain.Synchronize(time);
                vehicle->Synchronize(time, brake_inputs, terrain);
                trailer->Synchronize(time, brake_inputs, terrain);
                terrain.Advance(step_size);
                vehicle->Advance(step_size);
                trailer->Advance(step_size);
            }

            for (const auto& body : system->GetBodies()) {
                body->SetPosDt(VNULL);
                body->SetAngVelLocal(VNULL);
                body->SetPosDt2(VNULL);
            }

            system->SetChTime(0.0);
        }

        auto vehicle_agent = chrono_types::make_shared<SynWheeledVehicleAgent>(vehicle.get());
        vehicle_agent->SetZombieVisualizationFiles("LRV/meshes/Polaris_chassis.obj",
                                                   "LRV/meshes/Polaris_wheel.obj",
                                                   "LRV/meshes/Polaris_tire.obj");
        vehicle_agent->SetNumWheels(4);
        syn_manager.AddAgent(vehicle_agent);

        auto trailer_agent = chrono_types::make_shared<SynTrailerAgent>(trailer);
        trailer_agent->SetZombieVisualizationFiles("LRV_Wagon/trailer_chassis.obj",
                                                   "LRV/meshes/Polaris_wheel.obj",
                                                   "LRV/meshes/Polaris_tire.obj");
        trailer_agent->SetNumWheels(2);
        syn_manager.AddAgent(trailer_agent);

        syn_manager.AddAgent(chrono_types::make_shared<SynRockAgent>(owned_rocks, chrono_data_path,
                                                                     /*visualize_zombies=*/false, rock_field_config));

        SynLog() << "Rank " << rank << " owns robot index " << robot_index << " and " << owned_rocks.size()
                 << " dynamic rocks.\n";
    } else {
        syn_manager.AddAgent(chrono_types::make_shared<SynEnvironmentAgent>(system));
        SynLog() << "Rank 0 is sensor/visualization only; robot physics starts on rank 1.\n";
    }

    for (int robot_rank = 1; robot_rank < num_ranks; robot_rank++) {
        if (robot_rank == rank)
            continue;

        syn_manager.AddZombie(chrono_types::make_shared<SynRockAgent>(
                                  std::vector<std::shared_ptr<ChBodyAuxRef>>{}, chrono_data_path, is_sensor_rank,
                                  rock_field_config),
                              AgentKey(robot_rank, 3));
    }

    syn_manager.Initialize(system);

    std::shared_ptr<ChSensorManager> sensor_manager;
    if (is_sensor_rank) {
        sensor_manager = chrono_types::make_shared<ChSensorManager>(system);
        sensor_manager->scene->AddPointLight({-100, 0, 25}, {5.0f, 5.0f, 5.0f}, 1000);
        sensor_manager->SetVerbose(false);
        Background background;
        background.mode = BackgroundMode::ENVIRONMENT_MAP;
        background.env_tex = amd_uw_data_path + sensor_star_map;
        sensor_manager->scene->SetBackground(background);

        auto global_camera_body = chrono_types::make_shared<ChBody>();
        global_camera_body->SetFixed(true);
        system->AddBody(global_camera_body);

        auto global_camera = chrono_types::make_shared<ChCameraSensor>(
            global_camera_body,
            global_camera_update_rate,
            ChFrame<double>(global_camera_position, SensorLookAtRotation(global_camera_position, global_camera_target)),
            global_camera_width,
            global_camera_height,
            global_camera_fov);
        global_camera->SetName("Global Camera");
        global_camera->PushFilter(
            chrono_types::make_shared<ChFilterVisualize>(global_camera_width, global_camera_height, "Global Camera"));
        sensor_manager->AddSensor(global_camera);
    }

    // Optional VSG visualization for selected ranks.
    if (owns_robot && cli.HasValueInVector<int>("vsg", rank)) {
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
        vsg_app->AttachVehicle(vehicle.get());
        vsg_app->AttachDriver(irr_driver.get());
        vsg_app->AttachTerrain(&terrain);
        vsg_app->Initialize();
        app.Set(vsg_app);
    }

    const int render_steps = static_cast<int>(std::ceil(render_step_size / step_size));
    const int motion_log_steps =
        motion_log_rate > 0 ? std::max(1, static_cast<int>(std::ceil(1.0 / (motion_log_rate * step_size)))) : 0;
    int step_number = 0;
    ChRealtimeStepTimer realtime_timer;
    const auto wall_start = std::chrono::high_resolution_clock::now();

    // Main simulation loop: exchange Synchrono state, update local dynamics, and render if enabled.
    while (app.IsOk() && syn_manager.IsOk()) {
        const double time = system->GetChTime();
        if (time >= end_time)
            break;

        if (owns_robot && step_number % render_steps == 0)
            app.Render();

        syn_manager.Synchronize(time);

        if (owns_robot) {
            driver->Synchronize(time);
            const DriverInputs driver_inputs = driver->GetInputs();
            terrain.Synchronize(time);
            vehicle->Synchronize(time, driver_inputs, terrain);
            trailer->Synchronize(time, driver_inputs, terrain);
            app.Synchronize(time, driver_inputs);

            driver->Advance(step_size);
            terrain.Advance(step_size);
            vehicle->Advance(step_size);
            trailer->Advance(step_size);
            app.Advance(step_size);

            trailer_bed->SetPos(trailer->GetChassis()->GetPos() + ChVector3d(0, 0, 0.01));
            trailer_bed->SetRot(trailer->GetChassis()->GetRot());

            if (motion_log_steps > 0 && step_number % motion_log_steps == 0) {
                const auto chassis = vehicle->GetChassisBody();
                const ChVector3d p = chassis->GetPos();
                const ChVector3d v = chassis->GetPosDt();
                const ChVector3d a = chassis->GetPosDt2();
                const ChVector3d w = chassis->GetAngVelParent();
                const ChVector3d chassis_contact = chassis->GetContactForce();
                double tire_force_sum = 0.0;
                double tire_force_z = 0.0;

                for (const auto& axle : vehicle->GetAxles()) {
                    for (const auto& wheel : axle->GetWheels()) {
                        const auto& tire = wheel->GetTire();
                        if (!tire)
                            continue;
                        const auto force = tire->ReportTireForce(&terrain).force;
                        tire_force_sum += VecNorm(force);
                        tire_force_z += force.z();
                    }
                }

                SynLog() << "motion rank=" << rank << " t=" << system->GetChTime() << " pos=(" << p.x() << ","
                         << p.y() << "," << p.z() << ") speed=" << VecNorm(v) << " accel=" << VecNorm(a)
                         << " ang_speed=" << VecNorm(w) << " chassis_contact=" << VecNorm(chassis_contact)
                         << " tire_force_sum=" << tire_force_sum << " tire_force_z=" << tire_force_z << "\n";
            }
        } else {
            terrain.Synchronize(time);
            terrain.Advance(step_size);
            system->DoStepDynamics(step_size);
        }

        if (sensor_manager)
            sensor_manager->Update();

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
