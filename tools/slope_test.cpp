#include <cstdio>
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"

using namespace chrono;

// Braked (spin-locked) rigid cylinder on a 10-deg incline. Sweep friction mu
// to see whether a bigger coefficient rescues the SMC static hold.
template <typename SysT>
double run(ChContactMethod method, float mu, double young = 2e7) {
    double ang = 10.0 * CH_DEG_TO_RAD;
    ChQuaterniond rot = QuatFromAngleY(ang);
    SysT sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    auto mat = [&]() {
        ChContactMaterialData d; d.mu = mu; d.cr = 0.0f; d.Y = (float)young;
        return d.CreateMaterial(method);
    };
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(20, 20, 0.5, 1000, true, true, mat());
    ground->SetFixed(true); ground->SetRot(rot); sys.AddBody(ground);
    auto tire = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.4089, 0.2286, 300, true, true, mat());
    tire->SetPos(rot.Rotate(ChVector3d(0, 0, 0.4089 + 0.02))); tire->SetRot(rot);
    sys.AddBody(tire);
    auto lock = chrono_types::make_shared<ChLinkMateGeneric>(false,false,false,false,true,false);
    lock->Initialize(tire, ground, ChFramed(tire->GetPos(), rot)); sys.AddLink(lock);
    ChVector3d p0 = tire->GetPos();
    double dt = 5e-4;
    for (int i = 0; i < (int)(3.0/dt); ++i) sys.DoStepDynamics(dt);
    return (tire->GetPos() - p0).Length();
}

int main() {
    printf("Braked rigid-cylinder tire, 10deg slope, 3s slide (m). NSC vs SMC as mu grows:\n");
    printf("%-8s %-14s %-14s\n", "mu", "NSC", "SMC");
    for (float mu : {0.5f, 0.9f, 1.5f, 3.0f, 10.0f}) {
        printf("%-8.1f %-14.4f %-14.4f\n", mu, run<ChSystemNSC>(ChContactMethod::NSC, mu),
               run<ChSystemSMC>(ChContactMethod::SMC, mu));
    }
    printf("\nSMC only, mu fixed at 3.0, raising Young's modulus (tangential stiffness):\n");
    printf("%-12s %-14s\n", "Young(Pa)", "SMC slide(m)");
    for (double Y : {2e7, 2e8, 2e9, 2e10}) printf("%-12.0e %-14.4f\n", Y, run<ChSystemSMC>(ChContactMethod::SMC, 3.0f, Y));
    return 0;
}
