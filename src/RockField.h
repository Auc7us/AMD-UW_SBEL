#pragma once

#include <memory>
#include <string>
#include <vector>

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChTerrain.h"

namespace amd_uw {

// How MANY rocks a rank gets is deliberately not in here. It is per-rank now (2-6), and
// every process -- including the sensor rank, which owns no rock field but has to size a
// zombie pool for everyone else's -- has to be able to derive any rank's count without
// being handed a config for it. RobotLayout::RocksPerRank is that one function; this
// struct carries only the things that are the same everywhere.
struct RockFieldConfig {
    double mesh_scale = 0.2;
    double density = 2500.0;
    double first_distance = 20.0;
    double distance_step = 30.0;
    double surface_clearance = 0.05;
};

std::shared_ptr<chrono::ChTriangleMeshConnected> LoadRockMesh(const std::string& filename,
                                                              bool load_uv,
                                                              double mesh_scale);

// Adds one robot's rock field. When `rock_top_heights` is non-null it is filled
// (parallel to the returned rocks) with each rock's height above its resting
// contact, i.e. the mesh's top-z after scaling/normalization. The arm bridge
// uses this to aim the gripper at each rock's actual top instead of a single
// hard-coded height that only fits one mesh.
std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> AddRockFields(
    chrono::ChSystem* system,
    chrono::vehicle::ChTerrain& terrain,
    const std::shared_ptr<chrono::ChContactMaterial>& rock_mat,
    const std::string& chrono_data_path,
    const std::string& amd_uw_data_path,
    int robot_index,
    int num_robots,
    double height_probe_z,
    const RockFieldConfig& config,
    std::vector<double>* rock_top_heights = nullptr,
    // Harvest cycle. The rank's whole lane rotates cycle * 30 deg counter-clockwise
    // about the site centre each time its collector dumps a load, so a later cycle
    // spawns its rocks on a fresh line. The RNG is seeded from it too, so each cycle
    // gets a different scatter instead of replaying the same one on a new bearing.
    int cycle = 0);

// Adds one builder's SEED HEAP: real, pickable rocks heaped outboard of its lane at
// BuilderPileCenter, in a ring so the pile reads as dumped rather than arranged.
//
// This is the only feedstock placed by the site. It exists to give the builder something
// to lay during its collector's first outbound leg, which is minutes of sim long;
// everything after it is delivered by that collector and found by the arm bridge's reach
// search, not by an index. So the order this returns them in carries no meaning.
//
// Every one of them is created FIXED with collision on. That is the whole point: a
// couple of dozen extra bodies per rank would otherwise each carry six DOF, generate
// contact pairs against the terrain and each other, and be integrated every 5e-4 s step
// for the entire run -- to sit perfectly still. Fixed, they are static colliders that
// cost the solver nothing, and LrvArm unfixes exactly one of them, at the moment its
// gripper locks on, and re-fixes it once it has been laid (see BuilderArmRosBridge).
// So at most one rock in the heap is ever a dynamic body.
//
// Mesh scale comes from `config` and must stay the rover's, because these rocks ride to
// the sensor rank on the same SynRockAgent, whose zombie bodies are all built at
// config.mesh_scale -- a different scale here would draw them the wrong size on rank 0.
std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> AddBuilderPileRocks(
    chrono::ChSystem* system,
    chrono::vehicle::ChTerrain& terrain,
    const std::shared_ptr<chrono::ChContactMaterial>& rock_mat,
    const std::string& chrono_data_path,
    const std::string& amd_uw_data_path,
    int builder_index,
    int num_builders,
    int rock_count,
    double height_probe_z,
    const RockFieldConfig& config);

}  // namespace amd_uw
