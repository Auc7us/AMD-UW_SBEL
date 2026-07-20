// Minimal MPI SynChrono demo for the AMD-UW Polaris JSON vehicle.
// Rank 0 is the global sensor/visualization rank. Robot physics starts on rank 1.

#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "src/MaterialUtils.h"
#include "src/RockField.h"
#include "src/RobotRig.h"
#include "src/SynAgents.h"

using namespace chrono;
using namespace chrono::sensor;
using namespace chrono::synchrono;
using namespace chrono::vehicle;
using namespace amd_uw;

namespace {

// Simulation defaults shared by all MPI ranks.
const ChContactMethod contact_method = ChContactMethod::SMC;

// SMC penalty contact integrates stiff normal forces, so it needs a finer step
// than NSC (which used 2e-3 / 1e-3). Mirrors the Python demo's 5e-4 SMC step.
double step_size = 5e-4;
double tire_step_size = 2.5e-4;
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

ChQuaternion<> SensorLookAtRotation(const ChVector3d& camera_pos, const ChVector3d& target_pos) {
    const ChVector3d forward = (target_pos - camera_pos).GetNormalized();
    ChMatrix33<> rot;
    rot.SetFromAxisX(forward, VECT_Y);
    return rot.GetQuaternion();
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
    cli.AddOption<bool>("Simulation", "no_sensor",
                        "Disable the sensor/render rank 0 (it just syncs) -- measure physics without rendering");
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
    const bool no_sensor = cli.CheckOption("no_sensor");
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

    ChSystemSMC sensor_system;
    sensor_system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    std::unique_ptr<RobotRig> robot;
    ChSystem* system = &sensor_system;
    if (owns_robot) {
        const int robot_index = rank - 1;
        robot = std::make_unique<RobotRig>(contact_method, rank, robot_index, num_robot_ranks, tire_step_size,
                                           render_step_size);
        system = robot->GetSystem();
    }

    // Lunar gravity. ChVehicle's constructor sets Earth gravity (-9.81) on the
    // system; at that pull the rover's tires bog deep into the soft SCM regolith
    // and it can't drive. This is a lunar rover, so use 1.62 m/s^2 -- ~6x less
    // normal load -> far less sinkage while keeping the traction/weight ratio. Set
    // here (after the vehicle ctor, before terrain + settle) so settling and the
    // whole run use lunar gravity, on every rank.
    const double lunar_gravity = 1.62;
    system->SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -lunar_gravity));

    // Deformable SCM (Soil Contact Model) terrain: a per-robot lunar-regolith lane
    // (30 m across x 450 m along) laid over that robot's driving corridor, so each
    // robot drives its whole mission on SCM and gets a genuinely different chunk of
    // the map. The height profile is cropped from terrain2.bmp along each corridor
    // (see tools/make_scm_crop.py -> data/terrain/terrain_scm_r{i}.bmp). On SCM the
    // tire traction/braking comes from the soil shear model (Bekker/Janosi), not the
    // SMC penalty friction that fails on rigid ground. Rank 0 (sensor/viz, no robot)
    // shows robot 1's lane so the overhead camera sees the action.
    struct ScmLane {
        const char* file;
        double ox, oy, yaw_deg, hmin, hmax;
    };
    static const ScmLane scm_lanes[] = {
        {"terrain/terrain_scm_r0.bmp", 190.526, -135.000, 330.0, -10.458, 8.961},  // robot 1 (rank 1)
        {"terrain/terrain_scm_r1.bmp", 110.000, 215.526, 60.0, -12.028, 3.597},    // robot 2 (rank 2)
    };
    constexpr int num_lanes = static_cast<int>(sizeof(scm_lanes) / sizeof(scm_lanes[0]));
    const int lane_idx = std::min(owns_robot ? (rank - 1) : 0, num_lanes - 1);
    const ScmLane& lane = scm_lanes[lane_idx];
    // 2 cm SCM so the grouser bricks actually bite the soil (cures the front-wheel slip
    // seen at 0.1 m, where the grid can't feel the grousers). A fine grid can't cover
    // the 450 m lane (OOM), so this is a short flat patch centered on each rank's robot
    // spawn. To restore the 450 m mission lane: scm_length=450, scm_width=30, delta=0.1,
    // SetReferenceFrame(lane...) + Initialize(amd_uw_data_path+lane.file, ..., lane.hmin, lane.hmax, 0.1).
    const int lane_ridx = owns_robot ? (rank - 1) : 0;
    const double scm_length = 40.0;   // along heading
    const double scm_width = 16.0;    // across heading
    const double scm_delta = 0.02;    // fine grid: grousers imprint / catch soil
    const double lane_hdg = lane.yaw_deg * CH_DEG_TO_RAD;
    const double lane_spawn_y = (lane_ridx - 0.5 * (num_robot_ranks - 1)) * 50.0;
    const double lane_ox = std::cos(lane_hdg) * (scm_length / 2.0 - 4.0);
    const double lane_oy = lane_spawn_y + std::sin(lane_hdg) * (scm_length / 2.0 - 4.0);

    SCMTerrain terrain(system);
    terrain.SetReferenceFrame(ChCoordsys<>(ChVector3d(lane_ox, lane_oy, terrain_height_offset), QuatFromAngleZ(lane_hdg)));
    // Chrono's validated VIPER/Curiosity lunar regolith (grouser-equivalent shear).
    terrain.SetSoilParameters(0.82e6, 0.14e4, 1.0, 0.017e4, 35.0, 1.78e-2, 2e8, 3e4);
    terrain.SetPlotType(SCMTerrain::PLOT_SINKAGE, 0.0, 0.10);
    // Small marker domain registered BEFORE Initialize so SCM doesn't build a null-body
    // default domain that later crashes; kept tiny since active nodes scale as 1/delta^2.
    auto scm_marker = chrono_types::make_shared<ChBody>();
    scm_marker->SetFixed(true);
    scm_marker->SetPos(ChVector3d(lane_ox, lane_oy, terrain_height_offset));
    system->AddBody(scm_marker);
    terrain.AddActiveDomain(scm_marker, VNULL, ChVector3d(0.6, 0.6, 0.6));
    terrain.Initialize(scm_length, scm_width, scm_delta);  // flat fine patch

    const double start_spacing = 50.0;
    // Probe from above the tallest possible terrain so the downward ray cast hits.
    const double height_probe_z = terrain_height_offset + terrain_max_height + terrain_height_probe_clearance;
    auto rock_mat = MakeContactMaterial(contact_method, 0.9f, 0.0f);
    VsgAppWrapper app;

    if (owns_robot) {
        robot->InitializeOnTerrain(terrain, rock_mat, chrono_data_path, amd_uw_data_path, start_spacing, height_probe_z,
                                   vehicle_start_clearance, seat_clearance, settle_time, step_size, rock_field_config);
        // (SCM per-wheel active domains are set inside InitializeOnTerrain, before settle.)

        auto vehicle_agent = chrono_types::make_shared<SynWheeledVehicleAgent>(robot->GetVehicle());
        vehicle_agent->SetZombieVisualizationFiles("LRV/meshes/Polaris_chassis.obj",
                                                   "LRV/meshes/Polaris_wheel.obj",
                                                   "LRV/meshes/LRVtire_red_m.obj");
        vehicle_agent->SetNumWheels(4);
        syn_manager.AddAgent(vehicle_agent);

        auto trailer_agent = chrono_types::make_shared<SynTrailerAgent>(robot->GetTrailer());
        trailer_agent->SetZombieVisualizationFiles("LRV_Wagon/trailer_chassis.obj",
                                                   "LRV/meshes/Polaris_wheel.obj",
                                                   "LRV/meshes/LRVtire_red_m.obj");
        trailer_agent->SetNumWheels(2);
        syn_manager.AddAgent(trailer_agent);

        syn_manager.AddAgent(chrono_types::make_shared<SynRockAgent>(robot->GetRocks(), chrono_data_path,
                                                                     /*visualize_zombies=*/false, rock_field_config));
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

    // The SolidWorks importer's embedded `import pychrono` (in LrvArm) resets
    // Chrono's global data paths to the library's compiled-in default. That breaks
    // the zombie vehicle mesh lookups in Initialize() below (they resolve
    // "vehicle/LRV/meshes/..." against the wrong root and segfault on the missing
    // OBJ). Re-assert our data paths right before zombie creation so the lookups
    // resolve against the AMD-UW vehicle data. No-op for the non-importer build.
    SetChronoDataPath(vehicle_data_path);
    SetVehicleDataPath(vehicle_data_path);

    syn_manager.Initialize(system);

    std::shared_ptr<ChSensorManager> sensor_manager;
    if (is_sensor_rank && !no_sensor) {
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
        vsg_app->AttachVehicle(robot->GetVehicle());
        vsg_app->AttachDriver(robot->GetDriver());
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
            robot->Synchronize(time, terrain);
            const DriverInputs driver_inputs = robot->GetDriverInputs();
            terrain.Synchronize(time);
            app.Synchronize(time, driver_inputs);

            terrain.Advance(step_size);
            robot->Advance(step_size);
            app.Advance(step_size);
            robot->LogMotionIfNeeded(step_number, motion_log_steps, terrain);
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
