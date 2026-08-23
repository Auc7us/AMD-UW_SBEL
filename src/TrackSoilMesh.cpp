#include "TrackSoilMesh.h"

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace amd_uw {

namespace {

// A box reaching below the shoe mid-plane is one the soil can see. On the M113 single-pin
// shoe the bottom pad spans z in [-0.03, 0] and each side pad [-0.01, +0.01], while the
// top pad and the guide horn both start exactly at z = 0 -- so this test separates them
// cleanly without naming any of them.
constexpr double kSoilFacingZ = -1e-6;

// Sweep-sphere radius 0: the mesh must occupy exactly the volume the box did, or the
// track's bearing area (and so its ground pressure) changes as a side effect of a change
// that was only supposed to be about representation.
constexpr double kSweepRadius = 0.0;

// One axis-aligned box as a closed 12-triangle mesh, in the shape's own frame.
std::shared_ptr<chrono::ChTriangleMeshConnected> BoxAsMesh(const chrono::ChVector3d& half) {
    auto mesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
    auto& v = mesh->GetCoordsVertices();
    auto& f = mesh->GetIndicesVertices();
    const double x = half.x(), y = half.y(), z = half.z();
    v = {{-x, -y, -z}, {+x, -y, -z}, {+x, +y, -z}, {-x, +y, -z},
         {-x, -y, +z}, {+x, -y, +z}, {+x, +y, +z}, {-x, +y, +z}};
    f = {{0, 2, 1}, {0, 3, 2},   // -z
         {4, 5, 6}, {4, 6, 7},   // +z
         {0, 1, 5}, {0, 5, 4},   // -y
         {2, 3, 7}, {2, 7, 6},   // +y
         {1, 2, 6}, {1, 6, 5},   // +x
         {3, 0, 4}, {3, 4, 7}};  // -x
    return mesh;
}

int MeshifyBody(const std::shared_ptr<chrono::ChBody>& body, chrono::ChSystem* system) {
    auto model = body->GetCollisionModel();
    if (!model)
        return 0;

    // Snapshot first: Clear() invalidates the instance list, and there is no API to remove
    // one shape from a model, so the whole model is rebuilt from this snapshot.
    const std::vector<chrono::ChCollisionShapeInstance> instances = model->GetShapeInstances();

    std::vector<std::pair<chrono::ChFramed, std::shared_ptr<chrono::ChCollisionShape>>> keep;
    std::vector<std::pair<chrono::ChFramed, std::shared_ptr<chrono::ChCollisionShape>>> converted;

    for (const auto& inst : instances) {
        if (inst.shape->GetType() != chrono::ChCollisionShape::Type::BOX) {
            keep.emplace_back(inst.frame, inst.shape);
            continue;
        }
        auto box = std::static_pointer_cast<chrono::ChCollisionShapeBox>(inst.shape);
        const chrono::ChVector3d half = box->GetHalflengths();
        // Lowest point of this box in the shoe's own frame.
        const double min_z = inst.frame.TransformPointLocalToParent(chrono::ChVector3d(0, 0, -half.z())).z();
        if (min_z >= kSoilFacingZ) {
            keep.emplace_back(inst.frame, inst.shape);  // inward-facing: leave it a box
            continue;
        }
        // Same material as the box carried, so friction and restitution are untouched.
        auto mesh_shape = chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(
            box->GetMaterial(), BoxAsMesh(half), /*is_static=*/false, /*is_convex=*/true, kSweepRadius);
        converted.emplace_back(inst.frame, mesh_shape);
    }

    if (converted.empty())
        return 0;

    const bool was_enabled = body->IsCollisionEnabled();
    model->Clear();
    for (const auto& k : keep)
        model->AddShape(k.second, k.first);
    for (const auto& c : converted)
        model->AddShape(c.second, c.first);

    // A rebuilt model is unknown to Bullet until it is bound again. Per body rather than
    // BindAll(): a whole-system rebind while the rig sits in resting contact re-detects
    // every existing penetration and delivers the full SMC penalty force in one step --
    // the impulse RobotRig::StartNextHarvestCycle documents having been bitten by.
    if (was_enabled && system && system->GetCollisionSystem())
        system->GetCollisionSystem()->BindItem(body);

    return static_cast<int>(converted.size());
}

}  // namespace

int MeshifyTrackShoeSoilContact(chrono::vehicle::ChTrackedVehicle* vehicle, chrono::ChSystem* system) {
    if (!vehicle)
        return 0;

    int converted = 0;
    std::size_t shoes = 0;
    for (int side = 0; side < 2; ++side) {
        auto track = vehicle->GetTrackAssembly(static_cast<chrono::vehicle::VehicleSide>(side));
        if (!track)
            continue;
        for (std::size_t i = 0; i < track->GetNumTrackShoes(); ++i) {
            auto shoe = track->GetTrackShoe(i);
            if (!shoe)
                continue;
            converted += MeshifyBody(shoe->GetShoeBody(), system);
            ++shoes;
        }
    }
    std::cout << "[track] meshified soil contact on " << shoes << " track shoes: " << converted
              << " box shapes -> convex triangle meshes (" << (converted * 12) << " triangles total).\n";
    return converted;
}

}  // namespace amd_uw
