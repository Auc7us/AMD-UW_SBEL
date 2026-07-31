// Minimal MPI SynChrono demo for the AMD-UW Polaris JSON vehicle.
// Rank 0 is the global sensor/visualization rank. Robot physics starts on rank 1.

#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChSolverAPGD.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/timestepper/ChTimestepper.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "src/BuilderRig.h"
#include "src/LrvArm.h"
#include "src/MaterialUtils.h"
#include "src/RockField.h"
#include "src/RobotLayout.h"
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
// Sim-time period of the rank-local performance probe (0 disables it).
double perf_log_period = 0.0;

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
const double builder_ride_height = 0.7;
// Site geometry (centre, work circle, builder orbit, collector ring) lives in
// src/RobotLayout.h so the placement of every rank's builder and collector comes
// from one source.
// Physics ranks register rover, trailer, rocks, tracked builder, then arm.
const int builder_arm_agent_id = 5;
const double rock_line_extension_width = 16.0;
const double rock_line_extension_end_margin = 8.0;

ChVector3d track_point(0.0, 0.0, 1.0);

ChQuaternion<> SensorLookAtRotation(const ChVector3d& camera_pos, const ChVector3d& target_pos) {
    const ChVector3d forward = (target_pos - camera_pos).GetNormalized();
    ChMatrix33<> rot;
    rot.SetFromAxisX(forward, VECT_Y);
    return rot.GetQuaternion();
}

double RockLineEndDistance(const RockFieldConfig& config) {
    return config.first_distance +
           (config.rocks_per_rank - 1) * config.distance_step;
}

double DistanceToTerrainEdge(const ChVector3d& origin,
                             const ChVector3d& forward,
                             double half_length,
                             double half_width) {
    double distance = std::numeric_limits<double>::infinity();
    if (forward.x() > 0)
        distance = std::min(distance, (half_length - origin.x()) / forward.x());
    else if (forward.x() < 0)
        distance = std::min(distance, (-half_length - origin.x()) / forward.x());

    if (forward.y() > 0)
        distance = std::min(distance, (half_width - origin.y()) / forward.y());
    else if (forward.y() < 0)
        distance = std::min(distance, (-half_width - origin.y()) / forward.y());
    return distance;
}

void AddOrbitVisualRing(ChSystem* system,
                        RigidTerrain& terrain,
                        double center_x,
                        double center_y,
                        double radius,
                        double height_probe_z,
                        const ChColor& color,
                        const std::string& name) {
    constexpr int segments = 180;
    constexpr double width = 0.12;
    constexpr double thickness = 0.04;
    constexpr double surface_offset = 0.08;
    const double segment_angle = CH_2PI / static_cast<double>(segments);
    const double segment_length =
        2.0 * radius * std::sin(0.5 * segment_angle) * 1.02;

    auto ring = chrono_types::make_shared<ChBody>();
    ring->SetName(name);
    ring->SetFixed(true);
    ring->EnableCollision(false);

    for (int segment = 0; segment < segments; ++segment) {
        const double angle = (segment + 0.5) * segment_angle;
        const double x = center_x + radius * std::cos(angle);
        const double y = center_y + radius * std::sin(angle);
        const double z =
            terrain.GetHeight(ChVector3d(x, y, height_probe_z)) +
            surface_offset;
        auto shape = chrono_types::make_shared<ChVisualShapeBox>(
            segment_length, width, thickness);
        shape->SetColor(color);
        ring->AddVisualShape(
            shape,
            ChFramed(
                ChVector3d(x, y, z),
                QuatFromAngleZ(angle + 0.5 * CH_PI)));
    }

    system->AddBody(ring);
}

// The final rocks and builder centers lie beyond the finite terrain2 heightmap.
// Continue each line with a narrow, flat rigid strip whose elevation matches the
// heightmap immediately inside its edge. The mapped surface itself is untouched.
void AddRockLineTerrainExtensions(RigidTerrain& terrain,
                                  const std::shared_ptr<ChContactMaterial>& material,
                                  int num_robots,
                                  double height_probe_z,
                                  double terrain_length,
                                  double terrain_width,
                                  const RockFieldConfig& config) {
    const double rock_line_end_distance = RockLineEndDistance(config);
    for (int robot_index = 0; robot_index < num_robots; ++robot_index) {
        const ChVector3d origin = InitialGroundPositionForRobot(robot_index, num_robots);
        const ChVector3d forward = RockLineForwardForRobot(robot_index, num_robots);
        const double edge_distance =
            DistanceToTerrainEdge(origin, forward, 0.5 * terrain_length, 0.5 * terrain_width);
        if (!std::isfinite(edge_distance) ||
            rock_line_end_distance <= edge_distance)
            continue;

        const double strip_start = edge_distance - 1.0;
        const double strip_end =
            rock_line_end_distance + rock_line_extension_end_margin;
        const double strip_length = strip_end - strip_start;
        const double strip_center_distance = 0.5 * (strip_start + strip_end);
        const ChVector3d strip_center = origin + forward * strip_center_distance;
        const ChVector3d edge_probe = origin + forward * (edge_distance - 2.0);
        const double strip_height =
            terrain.GetHeight(ChVector3d(edge_probe.x(), edge_probe.y(), height_probe_z));
        const double heading = InitialHeadingRadForRobot(robot_index, num_robots);

        auto extension = terrain.AddPatch(
            material,
            ChCoordsys<>(ChVector3d(strip_center.x(), strip_center.y(), strip_height), QuatFromAngleZ(heading)),
            strip_length, rock_line_extension_width);
        extension->SetColor(ChColor(0.55f, 0.55f, 0.52f));
        terrain.BindPatch(extension);
        ApplyMaterialToVisualShapes(extension->GetGroundBody(), CreateLunarHapkeMaterial());
    }
}

// Rank-local performance probe.
//
// Two things make the naive measurement useless here. First, the demo only
// printed a *cumulative* wall/sim average, which smears the moment a cost starts
// growing across the whole run. Second, ChSystem's own timers are reset at the
// top of every DoStepDynamics, so they report the last step only and cannot be
// differenced across a window -- they have to be summed every step.
//
// So: accumulate wall clock per loop section and sum Chrono's per-step timers,
// then report the window totals as a fraction of the window's sim time (i.e.
// each number is that section's own contribution to wall/sim).
struct PerfAccum {
    double syn = 0.0;        // SynChrono MPI exchange
    double robot_sync = 0.0; // rover Synchronize + terrain + VSG Synchronize
    double bldr_sync = 0.0;  // builder Synchronize (ROS bridges + vehicle sync)
    double robot_adv = 0.0;  // rover Advance (owns the main system's step)
    double bldr_adv = 0.0;   // builder Advance (owns the builder system's step)
    double sensor = 0.0;     // Chrono::Sensor update
    double render = 0.0;     // VSG render
    double spin = 0.0;       // real-time throttle
    // Summed per-step Chrono timers for the rank's one shared system.
    double sys_step = 0.0;
    double sys_coll = 0.0;
    double sys_broad = 0.0;
    double sys_narrow = 0.0;
    double sys_setup = 0.0;
    double sys_solve = 0.0;
    double sys_update = 0.0;

    void Reset() { *this = PerfAccum(); }

    void AddSystemStep(const ChSystem* sys) {
        if (!sys)
            return;
        sys_step += sys->GetTimerStep();
        sys_coll += sys->GetTimerCollision();
        sys_broad += sys->GetTimerCollisionBroad();
        sys_narrow += sys->GetTimerCollisionNarrow();
        sys_setup += sys->GetTimerSetup();
        sys_solve += sys->GetTimerLSsolve();
        sys_update += sys->GetTimerUpdate();
    }
};

// Wall-clock stopwatch that adds its own lifetime to an accumulator field.
class ScopedTimer {
  public:
    explicit ScopedTimer(double& sink) : m_sink(sink), m_start(std::chrono::steady_clock::now()) {}
    ~ScopedTimer() { m_sink += std::chrono::duration<double>(std::chrono::steady_clock::now() - m_start).count(); }

  private:
    double& m_sink;
    std::chrono::steady_clock::time_point m_start;
};

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
    cli.AddOption<double>("Diagnostics", "perf_log",
                          "Sim-time period (s) of the per-rank instantaneous cost breakdown (0 disables it)",
                          std::to_string(perf_log_period));
    cli.AddOption<bool>("Diagnostics", "builder_no_arm", "Build the builder without its manipulator (cost bisection)");
    cli.AddOption<std::string>("Diagnostics", "solver", "Robot-rank solver: bb, apgd, or default", "bb");
    cli.AddOption<int>("Diagnostics", "solver_iterations", "Max solver iterations for the robot ranks", "100");
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
    perf_log_period = cli.GetAsType<double>("perf_log");
    BuilderRig::Options builder_options;
    builder_options.with_arm = !cli.CheckOption("builder_no_arm");
    const bool no_sensor = cli.CheckOption("no_sensor");
    const std::string solver_name = cli.GetAsType<std::string>("solver");
    const int solver_iterations = cli.GetAsType<int>("solver_iterations");
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
        SynLog() << "Site centre=(" << site_center_x << ", " << site_center_y << "); work circle="
                 << work_circle_radius << " m; builder orbit=" << builder_path_radius
                 << " m; collector ring=" << robot_start_radius << " m.\n";
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
    std::unique_ptr<BuilderRig> builder;
    ChSystem* system = &sensor_system;
    if (owns_robot) {
        const int robot_index = rank - 1;
        robot = std::make_unique<RobotRig>(contact_method, rank, robot_index, num_robot_ranks, tire_step_size,
                                           render_step_size);
        system = robot->GetSystem();
    }

    // Use lunar gravity on every rank for settling and the full simulation.
    const double lunar_gravity = 1.62;
    system->SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -lunar_gravity));

    // Apollo-site rigid height-map terrain, matching the setup from commit 29723a3.
    RigidTerrain terrain(system);
    auto ground_mat = MakeContactMaterial(contact_method, 0.9f, 0.0f);
    const ChCoordsys<> terrain_csys(ChVector3d(0.0, 0.0, terrain_height_offset), QUNIT);
    auto ground = terrain.AddPatch(ground_mat, terrain_csys, amd_uw_data_path + terrain_heightmap_file,
                                   terrain_length, terrain_width, terrain_min_height, terrain_max_height);
    ground->SetColor(ChColor(0.55f, 0.55f, 0.52f));
    terrain.Initialize();
    ApplyMaterialToVisualShapes(ground->GetGroundBody(), CreateLunarHapkeMaterial());

    // Bind the rigid patch now so the pre-step height probes used to place the
    // vehicle, trailer, and rocks hit the actual terrain surface.
    system->GetCollisionSystem()->BindAll();

    // Probe from above the tallest possible terrain so the downward ray cast hits.
    const double height_probe_z = terrain_height_offset + terrain_max_height + terrain_height_probe_clearance;
    AddOrbitVisualRing(system, terrain, site_center_x, site_center_y, work_circle_radius, height_probe_z,
                       ChColor(0.95f, 0.75f, 0.10f), "work_circle_30m");
    AddOrbitVisualRing(system, terrain, site_center_x, site_center_y, builder_path_radius, height_probe_z,
                       ChColor(0.10f, 0.65f, 0.95f), "builder_path_40m");
    AddOrbitVisualRing(system, terrain, site_center_x, site_center_y, robot_start_radius, height_probe_z,
                       ChColor(0.20f, 0.90f, 0.35f), "collector_ring_50m");
    AddRockLineTerrainExtensions(terrain, ground_mat, num_robot_ranks, height_probe_z, terrain_length, terrain_width,
                                 rock_field_config);
    auto rock_mat = MakeContactMaterial(contact_method, 0.9f, 0.0f);
    VsgAppWrapper app;

    if (owns_robot) {
        robot->InitializeOnTerrain(terrain, rock_mat, chrono_data_path, amd_uw_data_path, height_probe_z,
                                   vehicle_start_clearance, seat_clearance, settle_time, step_size, rock_field_config);
    }

    // The arm importer above resets Chrono's global vehicle-data path. Restore it
    // before the full M113 model resolves its JSON and mesh assets. Each physics
    // rank owns one complete builder, and that builder lives in the SAME system as
    // that rank's rover, rocks, and terrain -- Chrono only produces contacts within
    // one system, so this is what lets a builder pick up its rank's rocks. Rank 0
    // receives the tracked-vehicle visualization through Synchrono.
    SetChronoDataPath(vehicle_data_path);
    SetVehicleDataPath(vehicle_data_path);

    if (owns_robot) {
        // The single-pin track needs the high-iteration Barzilai-Borwein VI solver
        // (the default lets shoes drift off the road wheels). Now that the builder
        // shares the rank's system, this is a system-wide choice and is made here
        // rather than inside BuilderRig, which must not reconfigure a world it does
        // not own. Mirrors demo_VEH_M113's SetChronoSolver(BARZILAIBORWEIN, ...).
        //
        // It is also a system-wide RISK: the rover's arm is a serial chain of
        // ChLinkMotorRotationAngle joints on a ChLinkLockLock weld, and it now shares
        // one iteration budget with ~130 track shoes. --solver selects it so that a
        // divergence can be attributed instead of guessed at; solver_iterations tunes
        // the budget without a rebuild.
        if (solver_name == "bb") {
            auto solver = chrono_types::make_shared<ChSolverBB>();
            solver->SetMaxIterations(solver_iterations);
            solver->SetOmega(0.8);
            solver->SetSharpnessLambda(1.0);
            system->SetSolver(solver);
            system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
        } else if (solver_name == "apgd") {
            auto solver = chrono_types::make_shared<ChSolverAPGD>();
            solver->SetMaxIterations(solver_iterations);
            system->SetSolver(solver);
            system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
        } else if (solver_name != "default") {
            if (rank == 1)
                std::cout << "[main] unknown --solver '" << solver_name
                          << "'; expected bb, apgd, or default. Using bb.\n";
            auto solver = chrono_types::make_shared<ChSolverBB>();
            solver->SetMaxIterations(solver_iterations);
            solver->SetOmega(0.8);
            solver->SetSharpnessLambda(1.0);
            system->SetSolver(solver);
            system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
        }
        if (rank == 1)
            std::cout << "[main] solver=" << solver_name << " iterations=" << solver_iterations << "\n";
    }

    if (owns_robot) {
        const int builder_index = robot->GetRobotIndex();
        const ChVector3d builder_ground = BuilderOrbitGroundPosition(builder_index, num_robot_ranks);
        const double builder_ground_height =
            terrain.GetHeight(ChVector3d(builder_ground.x(), builder_ground.y(), height_probe_z));
        const double builder_z = builder_ground_height + builder_ride_height;
        const double builder_heading =
            BuilderOrbitHeadingRad(builder_index, num_robot_ranks);
        builder = std::make_unique<BuilderRig>(
            builder_index + 1, system, amd_uw_data_path,
            ChCoordsys<>(ChVector3d(builder_ground.x(), builder_ground.y(), builder_z),
                         QuatFromAngleZ(builder_heading)),
            builder_options);
    }

    if (owns_robot) {
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

        syn_manager.AddAgent(chrono_types::make_shared<SynTrackedVehicleAgent>(
            builder->GetVehicle(), amd_uw_data_path + "synchrono/vehicle/M113.json"));
        syn_manager.AddAgent(chrono_types::make_shared<SynBuilderArmAgent>(
            builder->GetArm() ? builder->GetArm()->GetBodies()
                              : std::vector<std::shared_ptr<ChBodyAuxRef>>{},
            amd_uw_data_path,
            /*visualize_zombies=*/false));

        const ChVector3d builder_pos = builder->GetPosition();
        SynLog() << "Rank " << rank << " builder at (" << builder_pos.x() << ", " << builder_pos.y() << ", "
                 << builder_pos.z() << ").\n";
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
        syn_manager.AddZombie(
            chrono_types::make_shared<SynBuilderArmAgent>(
                std::vector<std::shared_ptr<ChBodyAuxRef>>{},
                amd_uw_data_path,
                is_sensor_rank),
            AgentKey(robot_rank, builder_arm_agent_id));
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
        // The builder is already in this system, so it renders with no extra
        // attachment and the rover chase camera keeps its original target.
        vsg_app->Initialize();
        app.Set(vsg_app);
    }

    const int render_steps = static_cast<int>(std::ceil(render_step_size / step_size));
    const int motion_log_steps =
        motion_log_rate > 0 ? std::max(1, static_cast<int>(std::ceil(1.0 / (motion_log_rate * step_size)))) : 0;
    int step_number = 0;
    ChRealtimeStepTimer realtime_timer;
    const auto wall_start = std::chrono::high_resolution_clock::now();
    const auto WallSeconds = [&wall_start]() {
        return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - wall_start).count();
    };
    PerfAccum perf_accum;
    double perf_window_start_wall = 0.0;
    double perf_window_start_sim = 0.0;
    double next_perf_log = perf_log_period;

    // Main simulation loop: exchange Synchrono state, update local dynamics, and render if enabled.
    while (app.IsOk() && syn_manager.IsOk()) {
        const double time = system->GetChTime();
        if (time >= end_time)
            break;

        if (owns_robot && step_number % render_steps == 0) {
            ScopedTimer timer(perf_accum.render);
            app.Render();
        }

        {
            ScopedTimer timer(perf_accum.syn);
            syn_manager.Synchronize(time);
        }

        if (owns_robot) {
            {
                ScopedTimer timer(perf_accum.robot_sync);
                robot->Synchronize(time, terrain);
            }
            {
                ScopedTimer timer(perf_accum.bldr_sync);
                // Keep the builder stationed inboard of its collector's CURRENT drop
                // point: both derive from the same rank lane angle, which advances one
                // step per harvest cycle.
                if (robot)
                    builder->SetStationAngle(
                        RankRayAngleRad(robot->GetRobotIndex(), num_robot_ranks, robot->GetHarvestCycle()));
                builder->Synchronize(time);
            }
            const DriverInputs driver_inputs = robot->GetDriverInputs();
            {
                ScopedTimer timer(perf_accum.robot_sync);
                terrain.Synchronize(time);
                app.Synchronize(time, driver_inputs);
                terrain.Advance(step_size);
            }
            // Every Synchronize is done; now advance subsystems, then step the one
            // shared system exactly once. The builder advances first because
            // robot->Advance() is what issues that DoStepDynamics.
            {
                ScopedTimer timer(perf_accum.bldr_adv);
                builder->Advance(step_size);
            }
            {
                ScopedTimer timer(perf_accum.robot_adv);
                robot->Advance(step_size);
                app.Advance(step_size);
            }
            robot->LogMotionIfNeeded(step_number, motion_log_steps, terrain);
        } else {
            ScopedTimer timer(perf_accum.robot_adv);
            terrain.Synchronize(time);
            terrain.Advance(step_size);
            system->DoStepDynamics(step_size);
        }

        if (sensor_manager) {
            ScopedTimer timer(perf_accum.sensor);
            sensor_manager->Update();
        }

        perf_accum.AddSystemStep(system);

        if (rank == 0 && step_number % 1000 == 0 && step_number > 0)
            SynLog() << "time=" << time << "  wall/sim=" << WallSeconds() / time << "\n";

        // Per-rank cost breakdown for the window just closed: which loop section,
        // which system, and what the builder was being asked to do at the time.
        if (perf_log_period > 0 && time >= next_perf_log) {
            next_perf_log = time + perf_log_period;
            const double wall_now = WallSeconds();
            const double window_wall = wall_now - perf_window_start_wall;
            const double window_sim = std::max(1e-12, time - perf_window_start_sim);
            perf_window_start_wall = wall_now;
            perf_window_start_sim = time;
            // Every field below is "wall seconds spent per sim second", so the
            // section numbers sum to inst_wall/sim.
            const double n = 1.0 / window_sim;

            std::ostringstream perf;
            perf.setf(std::ios::fixed);
            perf.precision(2);
            perf << "[perf] t=" << time << " inst_wall/sim=" << (window_wall * n)
                 << " total_wall/sim=" << (wall_now / time) << "\n";
            perf << "       sect  syn=" << (perf_accum.syn * n) << " rsync=" << (perf_accum.robot_sync * n)
                 << " bsync=" << (perf_accum.bldr_sync * n) << " radv=" << (perf_accum.robot_adv * n)
                 << " badv=" << (perf_accum.bldr_adv * n) << " sensor=" << (perf_accum.sensor * n)
                 << " render=" << (perf_accum.render * n) << "\n";
            perf << "       chtim step=" << (perf_accum.sys_step * n) << " coll=" << (perf_accum.sys_coll * n)
                 << " [broad=" << (perf_accum.sys_broad * n) << " narrow=" << (perf_accum.sys_narrow * n)
                 << "] setup=" << (perf_accum.sys_setup * n) << " solve=" << (perf_accum.sys_solve * n)
                 << " upd=" << (perf_accum.sys_update * n) << "\n";
            if (robot) {
                // Rover drive telemetry: what the controller asked for, what the
                // traction guard allowed, and how often the guard was saturated. At
                // lunar gravity the guard only has mu*g ~ 1.3 m/s^2 of lateral grip,
                // so an over-ambitious target speed shows up here as throttle being
                // zeroed and brake being added on most steps.
                const DriverInputs& raw = robot->GetRawDriverInputs();
                const DriverInputs& guarded = robot->GetGuardedDriverInputs();
                perf << "       rover speed=" << robot->GetSpeed() << " cmd(str/thr/brk)=" << raw.m_steering << "/"
                     << raw.m_throttle << "/" << raw.m_braking << " guarded=" << guarded.m_steering << "/"
                     << guarded.m_throttle << "/" << guarded.m_braking
                     << " guard_limited=" << (100.0 * robot->GetGuardLimitFraction()) << "%\n";
                robot->ResetGuardStats();
            }
            if (builder) {
                const DriverInputs& inputs = builder->GetDriverInputs();
                const ChVector3d builder_pos = builder->GetPosition();
                perf << "       state nb=" << system->GetNumBodiesActive() << " nc=" << system->GetNumContacts()
                     << " steer=" << inputs.m_steering << " thr=" << inputs.m_throttle
                     << " brk=" << inputs.m_braking << " speed=" << builder->GetSpeed() << " r="
                     << std::hypot(builder_pos.x() - site_center_x, builder_pos.y() - site_center_y)
                     << " z=" << builder_pos.z() << " shoe_dmax=" << builder->GetMaxShoeDistance() << "\n";
            }
            SynLog() << perf.str();
            perf_accum.Reset();
        }

        {
            ScopedTimer timer(perf_accum.spin);
            realtime_timer.Spin(step_size);
        }
        step_number++;
    }

    syn_manager.QuitSimulation();
    return 0;
}
