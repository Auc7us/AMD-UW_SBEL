// Minimal MPI SynChrono demo for the AMD-UW Polaris JSON vehicle.
// Run with two ranks to see two synchronized Polaris vehicles on one flat terrain.

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/core/ChTypes.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::synchrono;
using namespace chrono::vehicle;

namespace {

// Simulation defaults shared by all MPI ranks.
const ChContactMethod contact_method = ChContactMethod::SMC;

double step_size = 3e-3;
double tire_step_size = 1e-3;
double end_time = 1000.0;
double heartbeat = 1e-2;
double render_step_size = 1.0 / 50.0;
double settle_time = 1.0;

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

// Command-line options used by this standalone AMD-UW demo.
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

    // Vehicle-relative paths point at AMD-UW data while vehicle JSON and meshes are created.
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

    const ChVector3d init_loc(0.0, rank * 4.0, 0.55);
    const ChQuaternion<> init_rot(1, 0, 0, 0);

    // Local dynamic vehicle for this rank.
    WheeledVehicle vehicle(GetVehicleDataFile("LRV/Polaris.json"), contact_method);
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

    vehicle.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Explicit 300 m x 300 m flat rigid ground plane.
    RigidTerrain terrain(vehicle.GetSystem());
    auto ground_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    ground_mat->SetFriction(0.9f);
    ground_mat->SetRestitution(0.01f);
    auto ground = terrain.AddPatch(ground_mat, CSYSNORM, 300.0, 300.0, 1.0);
    ground->SetColor(ChColor(0.45f, 0.48f, 0.52f));
    ground->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 300.0f, 300.0f);
    terrain.Initialize();

    DriverWrapper driver(vehicle);
    VsgAppWrapper app;

    // Interactive keyboard driver. The wrapper above keeps the vehicle braked until input arrives.
    auto irr_driver = chrono_types::make_shared<ChInteractiveDriver>(vehicle);
    irr_driver->SetSteeringDelta(render_step_size / 1.0);
    irr_driver->SetThrottleDelta(render_step_size / 1.0);
    irr_driver->SetBrakingDelta(render_step_size / 0.3);
    irr_driver->Initialize();
    driver.Set(irr_driver);

    // Optional pre-run settling under full brake before Synchrono state exchange begins.
    if (settle_time > 0) {
        DriverInputs brake_inputs = driver.GetInputs();
        const int settle_steps = static_cast<int>(std::ceil(settle_time / step_size));

        for (int i = 0; i < settle_steps; i++) {
            const double time = vehicle.GetSystem()->GetChTime();
            terrain.Synchronize(time);
            vehicle.Synchronize(time, brake_inputs, terrain);
            terrain.Advance(step_size);
            vehicle.Advance(step_size);
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
    syn_manager.Initialize(vehicle.GetSystem());

    // Optional VSG visualization for selected ranks. VSG runtime assets live in Chrono's data tree,
    // so switch the generic Chrono data path only after AMD-UW vehicle assets are loaded.
    if (cli.HasValueInVector<int>("vsg", rank)) {
        SetChronoDataPath("/home/chrono-user/mountdir/chrono/build/data/");
        auto vsg_app = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
        vsg_app->SetWindowTitle("AMD-UW SynChrono Polaris Flat Demo");
        vsg_app->SetWindowSize(1280, 800);
        vsg_app->SetWindowPosition(100, 100);
        vsg_app->SetChaseCamera(track_point, 8.0, 0.75);
        vsg_app->SetChaseCameraPosition(ChVector3d(-7.0, 4.0, 2.0));
        vsg_app->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
        vsg_app->SetLightIntensity(1.0f);
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

        const DriverInputs driver_inputs = driver.GetInputs();

        syn_manager.Synchronize(time);
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        app.Synchronize(time, driver_inputs);

        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        app.Advance(step_size);

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
