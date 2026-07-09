#include <cstdio>
#include <string>
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"

using namespace chrono;
using namespace chrono::vehicle;

// Full Polaris under SMC on a flat patch with gravity tilted by `ang_deg`
// (physically identical to a slope, no placement artifacts). Full brake the
// whole time. Reports longitudinal drift + final speed after settle.
struct Res { double drift; double speed; };

Res run(const std::string& tire_json, double ang_deg) {
    double g = 9.81, a = ang_deg * CH_DEG_TO_RAD;

    WheeledVehicle vehicle(GetVehicleDataFile("LRV/Polaris.json"), ChContactMethod::SMC);
    vehicle.Initialize(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT));
    vehicle.GetChassis()->SetFixed(false);
    auto sys = vehicle.GetSystem();
    sys->SetGravitationalAcceleration(ChVector3d(g * std::sin(a), 0, -g * std::cos(a)));

    auto engine = ReadEngineJSON(GetVehicleDataFile("LRV/Polaris_EngineSimpleMap.json"));
    auto trans = ReadTransmissionJSON(GetVehicleDataFile("LRV/Polaris_AutomaticTransmissionSimpleMap.json"));
    vehicle.InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, trans));

    for (auto& axle : vehicle.GetAxles())
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(GetVehicleDataFile(tire_json));
            vehicle.InitializeTire(tire, wheel, VisualizationType::NONE);
            tire->SetStepsize(2.5e-4);
        }

    RigidTerrain terrain(sys);
    ChContactMaterialData md; md.mu = 0.9f; md.cr = 0.0f; md.Y = 2e7f;
    auto patch = terrain.AddPatch(md.CreateMaterial(ChContactMethod::SMC),
                                  ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 200, 200);
    terrain.Initialize();

    DriverInputs in; in.m_steering = 0; in.m_throttle = 0; in.m_braking = 1.0;
    double dt = 5e-4, t = 0, settle = 1.0, obs = 3.0;
    double x0 = 0; bool got_x0 = false;
    while (t < settle + obs) {
        terrain.Synchronize(t);
        vehicle.Synchronize(t, in, terrain);
        terrain.Advance(dt);
        vehicle.Advance(dt);
        t += dt;
        if (!got_x0 && t >= settle) { x0 = vehicle.GetChassis()->GetPos().x(); got_x0 = true; }
    }
    double drift = vehicle.GetChassis()->GetPos().x() - x0;
    double speed = vehicle.GetChassis()->GetBody()->GetPosDt().Length();
    return {drift, speed};
}

int main(int argc, char** argv) {
    SetChronoDataPath(std::string(UW_AMD_VEHICLE_DATA_DIR) + "/");
    SetVehicleDataPath(std::string(UW_AMD_VEHICLE_DATA_DIR) + "/");
    struct T { const char* name; const char* json; };
    T tires[] = {
        {"RigidTire (current)", "LRV/Polaris_RigidTire.json"},
        {"TMeasy",              "LRV/Polaris_TMeasyTire.json"},
    };
    double angles[] = {5.0, 10.0, 15.0, 20.0};
    printf("Polaris full brake under SMC, slope via tilted gravity. drift over 3s (final speed):\n");
    printf("%-22s", "tire model");
    for (double ang : angles) printf(" %-20.0f", ang);
    printf("   (deg)\n");
    for (auto& tr : tires) {
        printf("%-22s", tr.name);
        for (double ang : angles) {
            Res r = run(tr.json, ang);
            char buf[64];
            snprintf(buf, sizeof(buf), "%6.2fm (%5.2f m/s)", r.drift, r.speed);
            printf(" %-20s", buf);
        }
        printf("\n");
    }
    return 0;
}
