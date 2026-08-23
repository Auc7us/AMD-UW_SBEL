#include "RockField.h"

#include <algorithm>
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
    chrono::vehicle::ChTerrain& terrain,
    const std::shared_ptr<chrono::ChContactMaterial>& rock_mat,
    const std::string& chrono_data_path,
    const std::string& amd_uw_data_path,
    int robot_index,
    int num_robots,
    double height_probe_z,
    const RockFieldConfig& config,
    std::vector<double>* rock_top_heights,
    int cycle) {
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
        // TRIANGLE MESH, not a convex hull built from the same file's vertices.
        //
        // rockN_hull.obj is already a convex hull, and the mesh is right here -- the old
        // ChCollisionShapeConvexHull(mat, mesh->GetCoordsVertices()) call threw the index
        // buffer away and had Bullet re-derive a polytope from the point cloud. That
        // costs nothing in contact quality (is_convex below keeps Bullet on the convex
        // algorithm) but it made the rocks invisible to SCM's GPU ray-cast backend, which
        // tests shape type literally and skips everything that is not TRIANGLEMESH.
        //
        // is_static=false: rocks are picked up and dropped. is_convex=true: these files
        // ARE hulls, so say so and keep the cheap, robust contact path.
        rock_ct_shapes[i] = chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(
            rock_mat, rock_collision_meshes[i], /*is_static=*/false, /*is_convex=*/true,
            /*radius=*/0.005);

        rock_vis_shapes[i] = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        rock_vis_shapes[i]->SetMesh(rock_visual_meshes[i]);
        rock_vis_shapes[i]->SetBackfaceCull(true);
        rock_vis_shapes[i]->AddMaterial(rock_vis_mat);
    }

    std::uniform_real_distribution<double> distance_jitter(0.0, 3.5);
    std::uniform_real_distribution<double> lateral_offset(-5.0, 5.0);
    std::uniform_real_distribution<double> yaw_offset(-chrono::CH_PI, chrono::CH_PI);

    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> rocks;
    std::mt19937 rng(20260621 + 4099 * robot_index + 104729 * cycle);
    const chrono::ChVector3d origin = InitialGroundPositionForRobot(robot_index, num_robots, cycle);
    const double heading = InitialHeadingRadForRobot(robot_index, num_robots, cycle);
    const chrono::ChVector3d forward(std::cos(heading), std::sin(heading), 0.0);
    const chrono::ChVector3d left(-std::sin(heading), std::cos(heading), 0.0);

    const int rocks_per_rank = RocksPerRank(robot_index);
    for (int i = 0; i < rocks_per_rank; i++) {
        const double distance = config.first_distance + i * config.distance_step + distance_jitter(rng);
        const chrono::ChVector3d xy = origin + forward * distance + left * lateral_offset(rng);
        const double terrain_z = terrain.GetHeight(chrono::ChVector3d(xy.x(), xy.y(), height_probe_z));
        const int shape_index = (robot_index * rocks_per_rank + i) % static_cast<int>(rock_visual_meshes.size());

        double mass;
        chrono::ChVector3d cog;
        chrono::ChMatrix33<> inertia;
        rock_collision_meshes[shape_index]->ComputeMassProperties(true, mass, cog, inertia);
        chrono::ChMatrix33<> principal_inertia_rot;
        chrono::ChVector3d principal_inertia;
        chrono::ChInertiaUtils::PrincipalInertia(inertia, principal_inertia, principal_inertia_rot);

        auto rock_body = chrono_types::make_shared<chrono::ChBodyAuxRef>();
        // Named so the trajectory recorder can identify it from a plain system sweep --
        // rock bodies are created here, spawned again on every harvest cycle, and never
        // held in any list the recorder is handed.
        rock_body->SetName("harvest_rock_r" + std::to_string(robot_index) + "_c" + std::to_string(cycle) + "_" +
                           std::to_string(i));
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

        if (rock_top_heights) {
            // Mesh is normalized so its bottom sits at the REF frame (min.z == 0);
            // max.z is therefore the rock's height above its resting contact.
            rock_top_heights->push_back(rock_visual_meshes[shape_index]->GetBoundingBox().max.z());
        }
    }

    return rocks;
}

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
    const RockFieldConfig& config) {
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> rocks;
    if (rock_count <= 0)
        return rocks;

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
    std::array<std::shared_ptr<chrono::ChTriangleMeshConnected>, 3> visual_meshes;
    std::array<std::shared_ptr<chrono::ChTriangleMeshConnected>, 3> collision_meshes;
    std::array<std::shared_ptr<chrono::ChCollisionShapeTriangleMesh>, 3> ct_shapes;
    std::array<std::shared_ptr<chrono::ChVisualShapeTriangleMesh>, 3> vis_shapes;
    double rock_radius = 0.0;
    for (size_t i = 0; i < rock_visual_obj_files.size(); i++) {
        visual_meshes[i] = LoadRockMesh(rock_visual_obj_files[i], true, config.mesh_scale);
        collision_meshes[i] = LoadRockMesh(rock_collision_obj_files[i], false, config.mesh_scale);
        // Triangle mesh, same reasoning as AddRockFields above: the hull OBJ is already
        // convex, the mesh is already loaded, and only TRIANGLEMESH is visible to SCM's
        // GPU ray-cast backend.
        ct_shapes[i] = chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(
            rock_mat, collision_meshes[i], /*is_static=*/false, /*is_convex=*/true,
            /*radius=*/0.005);
        vis_shapes[i] = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        vis_shapes[i]->SetMesh(visual_meshes[i]);
        vis_shapes[i]->SetBackfaceCull(true);
        vis_shapes[i]->AddMaterial(rock_vis_mat);
        for (const auto& v : visual_meshes[i]->GetCoordsVertices())
            rock_radius = std::max(rock_radius, std::hypot(v.x(), v.y()));
    }

    // Heaped, not scattered: the rocks of one heap sit in a ring around its centre so it
    // reads as a dumped pile. But they must not CROWD, or the gripper closing on one
    // fouls its neighbours -- the jaws are 1x geometry on a 2x arm and open to 0.388 m
    // separation, so each rock needs clear air on both sides. Size the ring from the gap
    // wanted between neighbouring surfaces rather than from a fudge factor: a first cut
    // at 1.35 * radius * n / pi left 0.30 m rocks only 0.07 m apart.
    constexpr double heap_surface_gap = 0.30;
    const int ring_n = std::max(2, wall_slots_per_pile);
    const double ring_radius = (2.0 * rock_radius + heap_surface_gap) / (2.0 * std::sin(chrono::CH_PI / ring_n));
    std::mt19937 rng(51413u + 6151u * static_cast<unsigned>(builder_index));
    std::uniform_real_distribution<double> yaw_offset(-chrono::CH_PI, chrono::CH_PI);

    for (int slot = 0; slot < rock_count; slot++) {
        const int pile = slot / wall_slots_per_pile;
        const int within = slot % wall_slots_per_pile;
        const chrono::ChVector3d center = BuilderPileCenter(builder_index, num_builders, pile);
        // Taken in ring order so consecutive slots draw from around the heap rather than
        // hollowing out one side of it.
        const double a = chrono::CH_2PI * within / ring_n;
        const chrono::ChVector3d xy = center + chrono::ChVector3d(std::cos(a), std::sin(a), 0.0) * ring_radius;
        const double terrain_z = terrain.GetHeight(chrono::ChVector3d(xy.x(), xy.y(), height_probe_z));
        const int shape_index = (builder_index * 3 + slot) % static_cast<int>(visual_meshes.size());

        double mass;
        chrono::ChVector3d cog;
        chrono::ChMatrix33<> inertia;
        collision_meshes[shape_index]->ComputeMassProperties(true, mass, cog, inertia);
        chrono::ChMatrix33<> principal_inertia_rot;
        chrono::ChVector3d principal_inertia;
        chrono::ChInertiaUtils::PrincipalInertia(inertia, principal_inertia, principal_inertia_rot);

        auto rock_body = chrono_types::make_shared<chrono::ChBodyAuxRef>();
        rock_body->SetName("seed_rock_b" + std::to_string(builder_index) + "_" + std::to_string(slot));
        // Fixed until the gripper locks on. See the header note.
        rock_body->SetFixed(true);
        rock_body->SetSleepingAllowed(false);
        rock_body->SetMass(config.density * mass);
        rock_body->SetInertiaXX(config.density * principal_inertia);
        rock_body->SetFrameCOMToRef(chrono::ChFrame<>(cog, principal_inertia_rot));
        rock_body->SetFrameRefToAbs(chrono::ChFrame<>(
            chrono::ChVector3d(xy.x(), xy.y(), terrain_z + config.surface_clearance),
            chrono::QuatFromAngleZ(yaw_offset(rng))));
        rock_body->AddCollisionShape(ct_shapes[shape_index]);
        rock_body->EnableCollision(true);
        rock_body->AddVisualShape(vis_shapes[shape_index]);
        system->AddBody(rock_body);
        rocks.push_back(rock_body);
    }

    return rocks;
}

}  // namespace amd_uw
