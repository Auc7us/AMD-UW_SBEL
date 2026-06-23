#include "RockField.h"

#include <array>
#include <cmath>
#include <random>
#include <stdexcept>

#include "MaterialUtils.h"
#include "RobotLayout.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/core/ChTypes.h"
#include "chrono/physics/ChMassProperties.h"
#include "chrono/utils/ChConstants.h"

namespace amd_uw {

namespace {

void NormalizeRockMeshOnGround(std::shared_ptr<chrono::ChTriangleMeshConnected> mesh) {
    const chrono::ChAABB bbox = mesh->GetBoundingBox();
    const double center_x = 0.5 * (bbox.min.x() + bbox.max.x());
    const double center_y = 0.5 * (bbox.min.y() + bbox.max.y());
    mesh->Transform(chrono::ChVector3d(-center_x, -center_y, -bbox.min.z()), chrono::ChMatrix33<>(1.0));
}

}  // namespace

std::shared_ptr<chrono::ChTriangleMeshConnected> LoadRockMesh(const std::string& filename,
                                                              bool load_uv,
                                                              double mesh_scale) {
    auto mesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(filename, false, load_uv);
    if (!mesh)
        throw std::runtime_error("Failed to load rock mesh: " + filename);

    mesh->Transform(chrono::ChVector3d(0, 0, 0), chrono::ChMatrix33<>(mesh_scale));
    mesh->Transform(chrono::ChVector3d(0, 0, 0), chrono::ChMatrix33<>(chrono::QuatFromAngleX(chrono::CH_PI_2)));
    mesh->RepairDuplicateVertices(1e-9);
    NormalizeRockMeshOnGround(mesh);
    return mesh;
}

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
    const RockFieldConfig& config) {
    const std::array<std::string, 3> rock_visual_obj_files = {
        chrono_data_path + "robot/curiosity/rocks/rock1.obj",
        chrono_data_path + "robot/curiosity/rocks/rock2.obj",
        chrono_data_path + "robot/curiosity/rocks/rock3.obj",
    };
    const std::array<std::string, 3> rock_collision_obj_files = {
        amd_uw_data_path + "rocks/curiosity_hulls/rock1_hull.obj",
        amd_uw_data_path + "rocks/curiosity_hulls/rock2_hull.obj",
        amd_uw_data_path + "rocks/curiosity_hulls/rock3_hull.obj",
    };

    auto rock_vis_mat = CreateLunarHapkeMaterial();
    std::array<std::shared_ptr<chrono::ChTriangleMeshConnected>, 3> rock_visual_meshes;
    std::array<std::shared_ptr<chrono::ChTriangleMeshConnected>, 3> rock_collision_meshes;
    std::array<std::shared_ptr<chrono::ChCollisionShapeTriangleMesh>, 3> rock_ct_shapes;
    std::array<std::shared_ptr<chrono::ChVisualShapeTriangleMesh>, 3> rock_vis_shapes;

    for (size_t i = 0; i < rock_visual_obj_files.size(); i++) {
        rock_visual_meshes[i] = LoadRockMesh(rock_visual_obj_files[i], true, config.mesh_scale);
        rock_collision_meshes[i] = LoadRockMesh(rock_collision_obj_files[i], false, config.mesh_scale);
        rock_ct_shapes[i] = chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(
            rock_mat, rock_collision_meshes[i], false, true, 0.005);

        rock_vis_shapes[i] = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        rock_vis_shapes[i]->SetMesh(rock_visual_meshes[i]);
        rock_vis_shapes[i]->SetBackfaceCull(true);
        rock_vis_shapes[i]->AddMaterial(rock_vis_mat);
    }

    std::uniform_real_distribution<double> distance_jitter(0.0, 3.5);
    std::uniform_real_distribution<double> lateral_offset(-15.0, 15.0);
    std::uniform_real_distribution<double> yaw_offset(-chrono::CH_PI, chrono::CH_PI);

    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> rocks;
    std::mt19937 rng(20260621 + 4099 * robot_index);
    const chrono::ChVector3d origin = InitialGroundPositionForRobot(robot_index, num_robots, start_spacing);
    const double heading = InitialHeadingDegForRobot(robot_index) * chrono::CH_DEG_TO_RAD;
    const chrono::ChVector3d forward(std::cos(heading), std::sin(heading), 0.0);
    const chrono::ChVector3d left(-std::sin(heading), std::cos(heading), 0.0);

    for (int i = 0; i < config.rocks_per_rank; i++) {
        const double distance = config.first_distance + i * config.distance_step + distance_jitter(rng);
        const chrono::ChVector3d xy = origin + forward * distance + left * lateral_offset(rng);
        const double terrain_z = terrain.GetHeight(chrono::ChVector3d(xy.x(), xy.y(), height_probe_z));
        const int shape_index = (robot_index * config.rocks_per_rank + i) % static_cast<int>(rock_visual_meshes.size());

        double mass;
        chrono::ChVector3d cog;
        chrono::ChMatrix33<> inertia;
        rock_collision_meshes[shape_index]->ComputeMassProperties(true, mass, cog, inertia);
        chrono::ChMatrix33<> principal_inertia_rot;
        chrono::ChVector3d principal_inertia;
        chrono::ChInertiaUtils::PrincipalInertia(inertia, principal_inertia, principal_inertia_rot);

        auto rock_body = chrono_types::make_shared<chrono::ChBodyAuxRef>();
        rock_body->SetFixed(false);
        rock_body->SetSleepingAllowed(true);
        rock_body->SetSleepTime(0.15f);
        rock_body->SetSleepMinLinVel(0.08f);
        rock_body->SetSleepMinAngVel(0.08f);
        rock_body->SetMass(config.density * mass);
        rock_body->SetInertiaXX(config.density * principal_inertia);
        rock_body->SetFrameCOMToRef(chrono::ChFrame<>(cog, principal_inertia_rot));
        rock_body->SetFrameRefToAbs(chrono::ChFrame<>(
            chrono::ChVector3d(xy.x(), xy.y(), terrain_z + config.surface_clearance),
            chrono::QuatFromAngleZ(yaw_offset(rng))));
        rock_body->AddCollisionShape(rock_ct_shapes[shape_index]);
        rock_body->EnableCollision(true);
        rock_body->AddVisualShape(rock_vis_shapes[shape_index]);
        system->AddBody(rock_body);
        rocks.push_back(rock_body);
    }

    return rocks;
}

}  // namespace amd_uw
