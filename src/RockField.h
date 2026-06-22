#pragma once

#include <memory>
#include <string>
#include <vector>

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

namespace amd_uw {

struct RockFieldConfig {
    int rocks_per_rank = 15;
    double mesh_scale = 0.3;
    double density = 2500.0;
    double first_distance = 15.0;
    double distance_step = 15.0;
    double surface_clearance = 0.05;
};

std::shared_ptr<chrono::ChTriangleMeshConnected> LoadRockMesh(const std::string& filename,
                                                              bool load_uv,
                                                              double mesh_scale);

std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> AddRockFields(
    chrono::ChSystem* system,
    chrono::vehicle::RigidTerrain& terrain,
    const std::shared_ptr<chrono::ChContactMaterial>& rock_mat,
    const std::string& chrono_data_path,
    const std::string& amd_uw_data_path,
    int robot_index,
    int num_robots,
    double start_spacing,
    double height_probe_z,
    const RockFieldConfig& config);

}  // namespace amd_uw
