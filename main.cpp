// Minimal MPI SynChrono demo for the AMD-UW Polaris JSON vehicle.
// Rank 0 is the global sensor/visualization rank. Robot physics starts on rank 1.

#include <mpi.h>

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
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
#include "chrono_sensor/filters/ChFilterSave.h"
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
// NOT const: --rock_first_distance / --rock_distance_step override the line geometry so
// a harvest cycle can be exercised in a fraction of the sim time. Defaults are unchanged.
RockFieldConfig rock_field_config;
const float global_camera_update_rate = 30.0f;
// 1080p. Same 16:9 aspect as the previous 1280x720, so the framing is unchanged and only
// the pixel count moves -- 0.92 Mpx to 2.07 Mpx, so budget ~2.25x the OptiX render cost
// on the sensor rank. That rank shares the SynChrono heartbeat with every robot rank, so
// if it becomes the pacing rank the whole job slows; global_camera_update_rate is the
// knob for that (24 Hz is still fine for video).
const unsigned int global_camera_width = 1920;
const unsigned int global_camera_height = 1080;
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
// Sim time for which the builder holds full brake and ignores every ROS command, so its
// single-pin track settles onto the terrain before a controller can move it.
const double builder_settle_time = 1.5;
// Site geometry (centre, work circle, builder orbit, collector ring) lives in
// src/RobotLayout.h so the placement of every rank's builder and collector comes
// from one source.
// Physics ranks register rover, trailer, rocks, tracked builder, arm, then trailer
// bed -- in that order. SynChronoManager::AddAgent hands out agent ids sequentially
// from 1 in call order, so these constants must match the AddAgent sequence below,
// and every AddZombie must use the same id as the agent it shadows.
const int trailer_agent_id = 2;
const int rock_agent_id = 3;
const int tracked_builder_agent_id = 4;
const int builder_arm_agent_id = 5;
const int trailer_bed_agent_id = 6;
const int rover_arm_agent_id = 7;
// Highest id a robot rank hands out -- ranks that draw no zombies register one inert
// placeholder per remote agent, so this must cover every AddAgent below.
const int last_agent_id = rover_arm_agent_id;

// Mesh directory and geometry scale per manipulator; the sensor rank must rebuild
// each arm exactly as its owning rank did.
const char* const builder_arm_shapes_dir = "m113_builder_arm/m113_builder_arm_shapes/";
const char* const rover_arm_shapes_dir = "lrv_robotarm/lrv_arm_shapes/";
const double builder_arm_geometry_scale = 2.0;
const double rover_arm_geometry_scale = 1.0;
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

// Seat the builder on a FITTED TERRAIN PLANE rather than a single height probe.
//
// The old placement probed the terrain at the chassis centre only and set the hull level
// (yaw only) at that height + builder_ride_height. Measured off terrain2.bmp, the ground
// under the builder's 5.4 x 2.7 m footprint tilts 2.4-9.5 degrees depending on where the
// lane puts it, so a level hull is out by 0.23-0.82 m across the footprint: the shoes at
// the high end start buried and the ones at the low end start in the air. Under SMC a
// buried shoe is a penalty force with nowhere to go, which is the same trap the pinned
// hull hits (see BuilderRig's constructor) reached by a different route.
//
// Fitting a plane and matching pitch and roll to it cuts the worst-case seating error to
// 0.019-0.077 m -- 6x to 20x better -- at every radius, which is the point: this is a fix
// for the seating, not for one lucky lane radius.
struct BuilderSeating {
    ChCoordsys<> pose;
    double level_error;  // worst seating error the old single-probe placement would give
    double plane_error;  // worst residual about the fitted plane
    double tilt_deg;
    double lift;
};

BuilderSeating SeatBuilderOnTerrainPlane(RigidTerrain& terrain,
                                         double center_x,
                                         double center_y,
                                         double yaw,
                                         double ride_height,
                                         double height_probe_z) {
    // Hull footprint in the chassis frame, from Chassis.obj. Note it is NOT centred on
    // the chassis reference (-4.903 .. +0.498), which is why the plane fit below has to
    // centre on the sample centroid rather than on the builder's own origin.
    constexpr double hull_x_min = -4.903;
    constexpr double hull_x_max = 0.498;
    constexpr double hull_y_half = 1.343;
    constexpr int nx = 9;
    constexpr int ny = 5;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    std::vector<ChVector3d> samples;
    samples.reserve(nx * ny);
    for (int i = 0; i < nx; ++i) {
        const double lx = hull_x_min + (hull_x_max - hull_x_min) * i / (nx - 1.0);
        for (int j = 0; j < ny; ++j) {
            const double ly = -hull_y_half + 2.0 * hull_y_half * j / (ny - 1.0);
            const double wx = center_x + lx * cos_yaw - ly * sin_yaw;
            const double wy = center_y + lx * sin_yaw + ly * cos_yaw;
            samples.emplace_back(wx, wy, terrain.GetHeight(ChVector3d(wx, wy, height_probe_z)));
        }
    }

    const double n = static_cast<double>(samples.size());
    double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
    for (const auto& s : samples) {
        mean_x += s.x();
        mean_y += s.y();
        mean_z += s.z();
    }
    mean_x /= n;
    mean_y /= n;
    mean_z /= n;

    double sxx = 0.0, sxy = 0.0, syy = 0.0, sxz = 0.0, syz = 0.0;
    for (const auto& s : samples) {
        const double ex = s.x() - mean_x, ey = s.y() - mean_y, ez = s.z() - mean_z;
        sxx += ex * ex;
        sxy += ex * ey;
        syy += ey * ey;
        sxz += ex * ez;
        syz += ey * ez;
    }
    const double det = sxx * syy - sxy * sxy;
    double slope_x = 0.0, slope_y = 0.0;
    if (std::abs(det) > 1e-12) {
        slope_x = (syy * sxz - sxy * syz) / det;
        slope_y = (sxx * syz - sxy * sxz) / det;
    }
    const auto plane_z = [&](double x, double y) {
        return mean_z + slope_x * (x - mean_x) + slope_y * (y - mean_y);
    };

    const double center_probe = terrain.GetHeight(ChVector3d(center_x, center_y, height_probe_z));
    double level_error = 0.0, plane_error = 0.0, max_residual = 0.0;
    for (const auto& s : samples) {
        level_error = std::max(level_error, std::abs(s.z() - center_probe));
        const double residual = s.z() - plane_z(s.x(), s.y());
        plane_error = std::max(plane_error, std::abs(residual));
        max_residual = std::max(max_residual, residual);
    }

    // Local Z on the plane normal, yaw preserved: SetFromAxisZ orthogonalises the
    // suggested X against the normal, so the tangential heading survives the tilt.
    ChMatrix33<> rot;
    rot.SetFromAxisZ(ChVector3d(-slope_x, -slope_y, 1.0).GetNormalized(),
                     ChVector3d(cos_yaw, sin_yaw, 0.0));

    BuilderSeating seating;
    // Lift by the largest bump above the fitted plane so the highest sample only just
    // touches, instead of starting inside a track shoe.
    seating.pose = ChCoordsys<>(
        ChVector3d(center_x, center_y, plane_z(center_x, center_y) + ride_height + max_residual),
        rot.GetQuaternion());
    seating.level_error = level_error;
    seating.plane_error = plane_error;
    seating.tilt_deg = std::atan(std::hypot(slope_x, slope_y)) * CH_RAD_TO_DEG;
    seating.lift = max_residual;
    return seating;
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
    // Radius is appended rather than baked into the caller's literal: the previous
    // names said 30m/40m/50m for rings that were actually at 30/35/40.
    ring->SetName(name + "_" + std::to_string(std::lround(radius)) + "m");
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

// A 10 x 10 m pad at the site centre, as four 5 x 5 m tiles. Visual only: fixed,
// collision-free, terrain untouched -- a marker, not a surface to drive on.
void AddCenterPad(ChSystem* system, RigidTerrain& terrain, double height_probe_z) {
    constexpr double tile = 5.0;         // one tile edge; four of them make 10 x 10
    constexpr double pad_thickness = 0.06;
    constexpr double surface_offset = 0.06;
    // Two dark greys in a checker, so this reads as four tiles and doubles as a 5 m
    // scale reference. Values look darker than they render: diffuse colour is linear
    // and the image is gamma-encoded, so 0.16 comes out ~100/255, the same as this
    // terrain. 0.03 measures ~45/255.
    const ChColor tile_dark(0.030f, 0.030f, 0.034f);
    const ChColor tile_light(0.055f, 0.055f, 0.060f);

    auto pad = chrono_types::make_shared<ChBody>();
    pad->SetName("center_pad_10x10");
    pad->SetFixed(true);
    pad->EnableCollision(false);

    // Highest terrain sample under the footprint: a flat slab on slope must clip one
    // edge or float at the other, and burying it hides the pad itself.
    double pad_z = -std::numeric_limits<double>::infinity();
    for (int ix = -2; ix <= 2; ++ix) {
        for (int iy = -2; iy <= 2; ++iy) {
            const double x = site_center_x + 0.25 * ix * 2.0 * tile;
            const double y = site_center_y + 0.25 * iy * 2.0 * tile;
            pad_z = std::max(pad_z, terrain.GetHeight(ChVector3d(x, y, height_probe_z)));
        }
    }
    pad_z += surface_offset;

    for (int ix = 0; ix < 2; ++ix) {
        for (int iy = 0; iy < 2; ++iy) {
            const double cx = site_center_x + (ix == 0 ? -0.5 : 0.5) * tile;
            const double cy = site_center_y + (iy == 0 ? -0.5 : 0.5) * tile;
            auto shape = chrono_types::make_shared<ChVisualShapeBox>(tile, tile, pad_thickness);
            shape->SetColor(((ix + iy) % 2 == 0) ? tile_dark : tile_light);
            pad->AddVisualShape(shape, ChFramed(ChVector3d(cx, cy, pad_z), QUNIT));
        }
    }

    system->AddBody(pad);
}

// Scenery rock clumps inside each rank's BUILDER REACH -- pyramid piles and scattered
// clusters, so a rank's work area reads as a real site rather than bare ground.
//
// VISUAL ONLY, and deliberately so. Everything goes on ONE fixed, collision-disabled body
// per rank, exactly as AddCenterPad does: a fixed body with collision off contributes no
// DOF to the solver and generates no contact pairs, so a few hundred rocks cost nothing
// but render time.
//
// Placed by the ARM's reach, not by an offset from something else. The builder parks
// tangentially at builder_path_radius with its arm base 2.5 m back along the hull, and the
// band proven in service is 2.78-4.44 m from that base (see RobotLayout). Clumps are put
// at 3.0-4.2 m from the base, fanned about the outward radial so they sit outboard of the
// hull rather than under it. Two useful consequences fall out: they clear the builder's
// tracks (which occupy builder_path_radius +/- 1.343 m), and because the arm base is
// already 2.5 m off the rank's ray, they land off the collector's radial approach path
// instead of in it.
//
// Every rank builds EVERY rank's clumps, like the rings, because the sensor rank has to
// see all of them. Placement is seeded per rank rather than randomly, so a robot rank's
// VSG and the global camera put the same rock in the same place.
void AddSpawnRockClumps(ChSystem* system,
                        RigidTerrain& terrain,
                        const std::string& chrono_data_path,
                        int num_robots,
                        double height_probe_z) {
    if (num_robots <= 0)
        return;

    // These are SCENERY -- background rubble, not feedstock. The rocks the builder
    // actually picks up are real bodies laid out by AddBuilderPileRocks, in heaps on the
    // outward radial at 36.6 m.
    //
    // Which is why this fan is now offset CLOCKWISE instead of centred on that radial.
    // Centred, it put decorative clumps within 1.3 m of the first working heap -- closer
    // than the sum of their spreads, so the two interpenetrated. The builder starts at
    // slot 0 and works counter-clockwise, so everything clockwise of its ray is ground it
    // never occupies; putting the rubble there keeps it out of both the heap line and the
    // lane while still reading as debris beside the machine.
    constexpr double reach_min = 3.4;
    constexpr double reach_max = 5.0;
    constexpr double arm_base_offset = -2.5;  // along the builder hull, from RobotLayout
    constexpr double fan_center = -55.0 * CH_DEG_TO_RAD;
    constexpr double fan_half_angle = 25.0 * CH_DEG_TO_RAD;
    constexpr int pyramid_clumps = 2;
    constexpr int cluster_clumps = 4;
    constexpr double cluster_spread = 0.9;

    // Three Curiosity rocks at three sizes, indexed [size][variant]. LoadRockMesh bakes the
    // scale into the mesh and drops its base to z=0, so a shape placed at terrain height
    // rests on the ground. Sizes are picked explicitly -- large on the bottom layer of a
    // pyramid, small on top -- which is why this is a 2D table and not one flat list.
    const std::array<std::string, 3> files = {
        chrono_data_path + "robot/curiosity/rocks/rock1.obj",
        chrono_data_path + "robot/curiosity/rocks/rock2.obj",
        chrono_data_path + "robot/curiosity/rocks/rock3.obj",
    };
    const std::array<double, 3> scales = {0.12, 0.20, 0.30};  // small, medium, large

    struct RockShape {
        std::shared_ptr<ChVisualShapeTriangleMesh> shape;
        double top = 0.0;     // scaled height, for stacking one layer on the next
        double radius = 0.0;  // scaled horizontal half-extent, for nesting within a layer
    };
    auto rock_material = CreateLunarHapkeMaterial();
    std::array<std::vector<RockShape>, 3> by_size;
    for (size_t s = 0; s < scales.size(); ++s) {
        for (const auto& file : files) {
            auto mesh = LoadRockMesh(file, true, scales[s]);
            if (!mesh)
                continue;
            RockShape entry;
            for (const auto& v : mesh->GetCoordsVertices()) {
                entry.top = std::max(entry.top, v.z());
                entry.radius = std::max(entry.radius, std::hypot(v.x(), v.y()));
            }
            entry.shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            entry.shape->SetMesh(mesh);
            entry.shape->SetBackfaceCull(true);
            entry.shape->SetMutable(false);
            entry.shape->AddMaterial(rock_material);
            by_size[s].push_back(entry);
        }
    }
    if (by_size[0].empty() || by_size[1].empty() || by_size[2].empty())
        return;

    auto scenery = chrono_types::make_shared<ChBody>();
    scenery->SetName("builder_reach_rock_clumps");
    scenery->SetFixed(true);
    scenery->EnableCollision(false);

    int placed = 0;
    for (int robot_index = 0; robot_index < num_robots; ++robot_index) {
        // Same seed on every rank, different per rank.
        std::mt19937 rng(90210u + 7919u * static_cast<unsigned>(robot_index));
        std::uniform_real_distribution<double> unit(-1.0, 1.0);
        std::uniform_real_distribution<double> yaw(-CH_PI, CH_PI);
        std::uniform_real_distribution<double> reach(reach_min, reach_max);
        std::uniform_real_distribution<double> fan(-fan_half_angle, fan_half_angle);
        std::uniform_int_distribution<int> base_layer_count(3, 4);
        std::uniform_int_distribution<int> mid_layer_count(2, 3);
        std::uniform_int_distribution<int> cluster_count(7, 9);

        const double ray = RankRayAngleRad(robot_index, num_robots);
        const ChVector3d outward(std::cos(ray), std::sin(ray), 0.0);
        // The builder parks tangent to its lane; its hull +X is the tangential direction.
        const double hull_heading = BuilderOrbitHeadingRad(robot_index, num_robots);
        const ChVector3d hull_axis(std::cos(hull_heading), std::sin(hull_heading), 0.0);
        const ChVector3d arm_base =
            BuilderOrbitGroundPosition(robot_index, num_robots) + hull_axis * arm_base_offset;

        auto pick = [&](int size) -> const RockShape& {
            std::uniform_int_distribution<int> v(0, static_cast<int>(by_size[size].size()) - 1);
            return by_size[size][v(rng)];
        };
        auto place = [&](const RockShape& r, const ChVector3d& xy, double z) {
            scenery->AddVisualShape(r.shape,
                                    ChFramed(ChVector3d(xy.x(), xy.y(), z), QuatFromAngleZ(yaw(rng))));
            ++placed;
        };
        // Clump centre: fanned clockwise of the outward radial, clear of the working heaps.
        auto clump_center = [&]() {
            const double angle = ray + fan_center + fan(rng);
            return arm_base + ChVector3d(std::cos(angle), std::sin(angle), 0.0) * reach(rng);
        };

        for (int clump = 0; clump < pyramid_clumps; ++clump) {
            const ChVector3d center = clump_center();
            const double ground = terrain.GetHeight(ChVector3d(center.x(), center.y(), height_probe_z));

            // Bottom layer: several LARGE rocks in a ring, spaced so neighbours just touch.
            const int n_base = base_layer_count(rng);
            double base_top = 0.0;
            double base_radius = 0.0;
            for (int i = 0; i < n_base; ++i) {
                const RockShape& r = pick(2);
                // Ring radius that makes n_base rocks of this size sit shoulder to shoulder.
                const double ring = (n_base > 1) ? r.radius / std::sin(CH_PI / n_base) : 0.0;
                const double a = CH_2PI * i / n_base + 0.3 * unit(rng);
                place(r, center + ChVector3d(std::cos(a), std::sin(a), 0.0) * ring, ground);
                base_top = std::max(base_top, r.top);
                base_radius = std::max(base_radius, ring);
            }

            // Middle layer: MEDIUM rocks nested into the hollows of the layer below, so they
            // sit at 70% of its height rather than perched on top of it.
            const int n_mid = mid_layer_count(rng);
            double mid_top = 0.0;
            const double mid_z = ground + 0.70 * base_top;
            for (int i = 0; i < n_mid; ++i) {
                const RockShape& r = pick(1);
                const double ring = 0.45 * base_radius;
                const double a = CH_2PI * i / n_mid + CH_PI / n_mid + 0.3 * unit(rng);
                place(r, center + ChVector3d(std::cos(a), std::sin(a), 0.0) * ring, mid_z);
                mid_top = std::max(mid_top, r.top);
            }

            // Capstone: one SMALL rock.
            place(pick(0), center + ChVector3d(0.12 * unit(rng), 0.12 * unit(rng), 0.0),
                  mid_z + 0.70 * mid_top);
        }

        for (int clump = 0; clump < cluster_clumps; ++clump) {
            const ChVector3d center = clump_center();
            const int n = cluster_count(rng);
            for (int i = 0; i < n; ++i) {
                // Mixed sizes lying about, each settled a little into the regolith.
                std::uniform_int_distribution<int> size(0, 2);
                const RockShape& r = pick(size(rng));
                const ChVector3d xy =
                    center + ChVector3d(cluster_spread * unit(rng), cluster_spread * unit(rng), 0.0);
                const double ground = terrain.GetHeight(ChVector3d(xy.x(), xy.y(), height_probe_z));
                place(r, xy, ground - 0.15 * r.top);
            }
        }
    }

    system->AddBody(scenery);
    SynLog() << "Builder-reach rock clumps: " << placed << " rocks on 1 fixed collision-free body ("
             << pyramid_clumps << " pyramids + " << cluster_clumps << " clusters x " << num_robots
             << " ranks).\n";
}

// Rocks already LAID on the work circle behind each builder -- the start of the wall it
// is building, so the site reads as work in progress rather than untouched ground.
//
// Deliberately not scattered: these are placed, so they sit exactly on
// work_circle_radius, evenly spaced by their own width, yaw aligned to the circle's
// tangent. That reads as deliberate construction; jitter would make it look like spill.
//
// "Behind" means clockwise of the rank's ray, because builders orbit counter-clockwise --
// so the laid section trails the machine that laid it.
//
// Visual only, on the same fixed collision-free body rationale as the clumps.
void AddPlacedWallRocks(ChSystem* system,
                        RigidTerrain& terrain,
                        const std::string& chrono_data_path,
                        int num_robots,
                        double height_probe_z) {
    if (num_robots <= 0)
        return;
    constexpr int rocks_per_rank = 5;
    // Scale and pitch BOTH match the rocks the builder actually lays (rock_field_config's
    // mesh_scale, wall_slot_pitch_rad), because these are the same course. The builder
    // starts laying at slot 0 -- exactly on the rank's ray -- and works counter-clockwise
    // from there; this is the section already behind it. At the old 0.30 scale and its own
    // independent spacing, the decoration's leading stone sat 0.48 m from slot 0 and the
    // first rock the builder laid landed on top of it.
    const double wall_rock_scale = rock_field_config.mesh_scale;

    auto material = CreateLunarHapkeMaterial();
    const std::array<std::string, 3> files = {
        chrono_data_path + "robot/curiosity/rocks/rock1.obj",
        chrono_data_path + "robot/curiosity/rocks/rock2.obj",
        chrono_data_path + "robot/curiosity/rocks/rock3.obj",
    };
    std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> shapes;
    for (const auto& file : files) {
        auto mesh = LoadRockMesh(file, true, wall_rock_scale);
        if (!mesh)
            continue;
        auto shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        shape->SetMesh(mesh);
        shape->SetBackfaceCull(true);
        shape->SetMutable(false);
        shape->AddMaterial(material);
        shapes.push_back(shape);
    }
    if (shapes.empty())
        return;

    auto wall = chrono_types::make_shared<ChBody>();
    wall->SetName("work_circle_placed_rocks");
    wall->SetFixed(true);
    wall->EnableCollision(false);

    int placed = 0;
    for (int robot_index = 0; robot_index < num_robots; ++robot_index) {
        const double ray = RankRayAngleRad(robot_index, num_robots);
        for (int i = 0; i < rocks_per_rank; ++i) {
            // Trailing the ray: i = 0 is the stone immediately before slot 0, running
            // clockwise from there, one full pitch clear of the builder's first slot.
            const double angle = ray - (i + 1) * wall_slot_pitch_rad;
            const double x = site_center_x + work_circle_radius * std::cos(angle);
            const double y = site_center_y + work_circle_radius * std::sin(angle);
            const double z = terrain.GetHeight(ChVector3d(x, y, height_probe_z));
            // Yaw follows the tangent so the course curves with the circle. Alternating
            // meshes keeps five identical stones from looking cloned.
            const auto& shape = shapes[(robot_index + i) % shapes.size()];
            wall->AddVisualShape(shape, ChFramed(ChVector3d(x, y, z),
                                                 QuatFromAngleZ(angle + CH_PI_2)));
            ++placed;
        }
    }

    system->AddBody(wall);
    SynLog() << "Work-circle placed rocks: " << placed << " on 1 fixed collision-free body ("
             << rocks_per_rank << " per rank x " << num_robots << " ranks, r=" << work_circle_radius
             << " m).\n";
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

        // VSG counterpart of --sensor_frame_dir.
        if (!m_frame_dir.empty() && m_frames_left > 0) {
            --m_frames_left;
            m_app->WriteImageToFile(m_frame_dir + "vsg_frame_" + std::to_string(m_frame_index++) + ".png");
        }
    }

    // Writes the first `count` rendered frames into `dir`, then stops.
    void SaveFrames(const std::string& dir, int count) {
        m_frame_dir = dir;
        m_frames_left = count;
    }

  private:
    std::shared_ptr<ChWheeledVehicleVisualSystemVSG> m_app;
    std::string m_frame_dir;
    int m_frames_left = 0;
    int m_frame_index = 0;
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
    // Divergence bisection: is the builder involved at all? Skipping it also skips its
    // two AddAgent calls, which SHIFTS every later SynChrono agent id, so a rank that
    // builds real zombies would mis-key them. Use with --no_sensor.
    cli.AddOption<bool>("Diagnostics", "no_builder",
                        "Omit the tracked builder entirely (divergence bisection; shifts SynChrono agent "
                        "ids, so use with --no_sensor)");
    // Separates the builder's DRIVE from its BUILD: the arm is only offered a pick while
    // the builder counts as parked, so suppressing that leaves it orbiting and idle.
    cli.AddOption<bool>("Diagnostics", "no_build",
                        "Builder drives its lane but never picks or places (cost/divergence "
                        "bisection)");
    cli.AddOption<std::string>("Diagnostics", "solver", "Robot-rank solver: bb, apgd, or default", "bb");
    cli.AddOption<int>("Diagnostics", "solver_iterations", "Max solver iterations for the robot ranks", "100");
    cli.AddOption<std::vector<int>>("VSG", "vsg", "MPI ranks that should open VSG visualization", "-1");
    cli.AddOption<bool>("Simulation", "no_sensor",
                        "Disable the sensor/render rank 0 (it just syncs) -- measure physics without rendering");
    // Save frames instead of opening a window, so either view can be checked from a
    // script with no display. The sensor rank draws zombies and VSG draws the real
    // bodies, so the two can legitimately differ.
    cli.AddOption<std::string>("Diagnostics", "sensor_frame_dir",
                               "Save global-camera frames as PNG to this directory instead of opening a window", "");
    // Shorten the harvest round trip for testing. The whole collector-to-builder hand-off
    // -- return leg, tangential run-in, dump, freeze, builder pick -- cannot be exercised
    // without a complete harvest cycle, and at the defaults that is about 400 s of SIM per
    // cycle, nearly all of it a loaded rover crawling back from 87 m at 0.2-0.5 m/s. That
    // is 2.5 hours of wall time per attempt at this machine's ~22x, which is not a
    // practical edit-test loop for anything downstream of the dump. These move the rocks
    // in without touching any default.
    cli.AddOption<double>("Diagnostics", "rock_first_distance",
                          "Metres from the drop point to the first rock (default 20)", "20.0");
    cli.AddOption<double>("Diagnostics", "rock_distance_step",
                          "Metres between successive rocks on a rank's line (default 30)", "30.0");
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
    const bool no_builder = cli.CheckOption("no_builder");
    const bool no_build = cli.CheckOption("no_build");
    const bool no_sensor = cli.CheckOption("no_sensor");
    const std::string sensor_frame_dir = cli.GetAsType<std::string>("sensor_frame_dir");
    const std::string solver_name = cli.GetAsType<std::string>("solver");
    const int solver_iterations = cli.GetAsType<int>("solver_iterations");
    rock_field_config.first_distance = cli.GetAsType<double>("rock_first_distance");
    rock_field_config.distance_step = cli.GetAsType<double>("rock_distance_step");
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
    // Only the sensor rank needs copies of other ranks' bodies, and only when its
    // camera exists. Rank 0 never opens a VSG window (that requires owning a robot).
    const bool draws_zombies = is_sensor_rank && !no_sensor;
    const bool owns_robot = (rank > 0);
    const int num_robot_ranks = std::max(0, num_ranks - 1);

    ChSystemSMC sensor_system;
    sensor_system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    std::unique_ptr<RobotRig> robot;
    std::unique_ptr<BuilderRig> builder;
    // Where the builder was seated, so the perf log can report how far it has slid.
    ChVector3d builder_spawn_pos(0.0, 0.0, 0.0);
    // The builder's feedstock. Declared out here because SynRockAgent holds the ADDRESS
    // of this vector for the whole run.
    std::vector<std::shared_ptr<ChBodyAuxRef>> builder_pile_rocks;
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
                       ChColor(0.95f, 0.75f, 0.10f), "work_circle");
    AddOrbitVisualRing(system, terrain, site_center_x, site_center_y, builder_path_radius, height_probe_z,
                       ChColor(0.10f, 0.65f, 0.95f), "builder_path");
    AddOrbitVisualRing(system, terrain, site_center_x, site_center_y, robot_start_radius, height_probe_z,
                       ChColor(0.20f, 0.90f, 0.35f), "collector_ring");
    AddCenterPad(system, terrain, height_probe_z);
    // AddSpawnRockClumps is deliberately NOT called. It puts ~90 decorative rocks per rank
    // in the builder's working area, and now that the builder works from a single seed heap
    // and then from whatever its collector actually delivers, that dressing reads as
    // feedstock the builder is ignoring. The function is kept -- it is the site-dressing
    // pass to re-enable for a wide establishing shot, where nothing is being picked up.
    AddPlacedWallRocks(system, terrain, chrono_data_path, num_robot_ranks, height_probe_z);
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

    if (owns_robot && !no_builder) {
        const int builder_index = robot->GetRobotIndex();
        const ChVector3d builder_ground = BuilderOrbitGroundPosition(builder_index, num_robot_ranks);
        const double builder_heading =
            BuilderOrbitHeadingRad(builder_index, num_robot_ranks);
        const BuilderSeating seating =
            SeatBuilderOnTerrainPlane(terrain, builder_ground.x(), builder_ground.y(), builder_heading,
                                      builder_ride_height, height_probe_z);
        SynLog() << "Rank " << rank << " builder seating: terrain tilt " << seating.tilt_deg
                 << " deg, worst error level=" << seating.level_error << " m -> plane="
                 << seating.plane_error << " m, lift=" << seating.lift << " m.\n";

        // The builder's own build plan: a course of slots to fill on the work circle, and
        // ONE seed heap to start filling them from. Everything after the seed comes from
        // this rank's collector -- see BuilderRig::SetDeliveredRockSource below and
        // RobotLayout's build-plan block.
        BuilderRig::Options plan_options = builder_options;
        const int wall_slot_count = BuilderWallSlotCount(num_robot_ranks);
        builder_pile_rocks =
            AddBuilderPileRocks(system, terrain, rock_mat, chrono_data_path, amd_uw_data_path, builder_index,
                                num_robot_ranks, builder_seed_rock_count, height_probe_z, rock_field_config);
        plan_options.seed_rocks = builder_pile_rocks;
        for (int slot = 0; slot < wall_slot_count; ++slot) {
            ChVector3d p = BuilderWallSlotPosition(builder_index, num_robot_ranks, slot);
            // Release height above the terrain at that slot: the rock is let go from rest
            // and drops the last little bit, so it seats itself instead of being pressed
            // into the regolith by the fingers.
            p.z() = terrain.GetHeight(ChVector3d(p.x(), p.y(), height_probe_z)) + 0.20;
            plan_options.wall_slots.push_back(p);
        }
        SynLog() << "Rank " << rank << " build plan: " << wall_slot_count << " wall slots at "
                 << wall_slot_pitch_m << " m pitch, " << builder_pile_rocks.size()
                 << " seed rocks in 1 heap, then " << harvest_rocks_per_load
                 << " per collector load every " << harvest_rocks_per_load << " slots; "
                 << wall_slot_count * wall_slot_pitch_rad * builder_path_radius << " m of lane to walk.\n";

        // Reach audit, at t=0, against the NOMINAL arm base for each slot. Every target in
        // this plan has to be inside the arm's envelope or the builder silently refuses
        // rocks it is parked beside -- and that only shows up once the machine has driven
        // to its first station, tens of minutes into a run. A first cut at the heap angle
        // put every heap 5.62 m out, past the 5.2 m guard, and nothing said so until then.
        // Bounds are LrvArm's scaled envelope: [1.0, 2.6] m * arm_geometry_scale 2.0.
        //
        // THREE things are audited, because there are three ways to be out of reach now:
        // the wall slot (inboard), the seed heap (outboard, over the slots it serves), and
        // the collector's drop point for each harvest cycle (outboard, over the slot
        // HarvestDropSlot puts it at). The last is the one that decides whether the
        // builder can actually eat what its collector brings.
        {
            constexpr double envelope_min = 2.0;
            constexpr double envelope_max = 5.2;
            auto arm_base_at_slot = [&](double slot) {
                // The hull leads its slot by arm_lead precisely so the ARM BASE lands on
                // the slot's own angle; see BuilderStationAngleRad.
                const double a = RankRayAngleRad(builder_index, num_robot_ranks) + slot * wall_slot_pitch_rad;
                return ChVector3d(site_center_x + builder_arm_base_radius_approx * std::cos(a),
                                  site_center_y + builder_arm_base_radius_approx * std::sin(a), 0.0);
            };
            double heap_min = 1e9, heap_max = 0.0, slot_min = 1e9, slot_max = 0.0;
            double drop_min = 1e9, drop_max = 0.0;
            for (int slot = 0; slot < wall_slot_count; ++slot) {
                const ChVector3d base = arm_base_at_slot(slot);
                const ChVector3d wall = BuilderWallSlotPosition(builder_index, num_robot_ranks, slot);
                slot_min = std::min(slot_min, (wall - base).Length());
                slot_max = std::max(slot_max, (wall - base).Length());
                if (slot < builder_seed_rock_count) {
                    const ChVector3d heap = BuilderPileCenter(builder_index, num_robot_ranks, 0);
                    heap_min = std::min(heap_min, (heap - base).Length());
                    heap_max = std::max(heap_max, (heap - base).Length());
                }
            }
            // Each load lands at HarvestDropSlot(c) and is eaten over the next
            // harvest_rocks_per_load slots, so check both ends of that run.
            for (int cycle = 0; HarvestDropSlot(cycle) < wall_slot_count; ++cycle) {
                // HarvestDropPoint, not InitialGroundPositionForRobot: the audit has to
                // measure where the ROCKS land, and the latter is where the tractor parks
                // -- 2.4 m of arc further on, so the rig's pour lip lands here.
                const ChVector3d drop = HarvestDropPoint(builder_index, num_robot_ranks, cycle);
                for (int k = 0; k < harvest_rocks_per_load; ++k) {
                    const double d = (drop - arm_base_at_slot(HarvestDropSlot(cycle) + k)).Length();
                    drop_min = std::min(drop_min, d);
                    drop_max = std::max(drop_max, d);
                }
            }
            SynLog() << "Rank " << rank << " reach audit: seed heap " << heap_min << "-" << heap_max
                     << " m, drop point " << drop_min << "-" << drop_max << " m, wall slot " << slot_min
                     << "-" << slot_max << " m (arm envelope " << envelope_min << "-" << envelope_max
                     << " m).\n";
            const bool out = heap_min < envelope_min || heap_max > envelope_max ||
                             slot_min < envelope_min || slot_max > envelope_max ||
                             drop_min < envelope_min || drop_max > envelope_max;
            if (out) {
                SynLog() << "Rank " << rank
                         << " BUILD PLAN OUT OF REACH: the builder will refuse targets it is parked "
                            "beside. Check builder_pile_radius / wall_slot_pitch_m / HarvestDropSlot / "
                            "the arm_lead convention in RobotLayout.h.\n";
            }
        }

        builder = std::make_unique<BuilderRig>(builder_index + 1, system, amd_uw_data_path, seating.pose,
                                               plan_options);
        builder_spawn_pos = builder->GetPosition();
        // Nothing may command this builder until its track has found equilibrium on the
        // terrain. The rover gets a careful settle and re-seat (RobotRig::Settle); the
        // builder never had one.
        builder->SetCommandEnableTime(builder_settle_time);
        builder->SetHullParkEnabled(!no_build);

        // The other half of the feedstock: whatever this rank's collector has delivered.
        // Wired here because main owns both rigs; neither knows about the other, and the
        // builder pulls rather than the rover pushing, so a load that lands while the arm
        // is mid-carry is simply seen on the next selection.
        if (robot) {
            RobotRig* rig = robot.get();
            builder->SetDeliveredRockSource([rig]() { return rig->GetDeliveredRocks(); });
        }
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

        // Pass the ADDRESS of the rig's live rock vector. Passing it by value took a
        // snapshot of the two cycle-0 rocks that never grew, so every rock spawned
        // by a later harvest cycle was transmitted to nobody and drawn nowhere.
        syn_manager.AddAgent(chrono_types::make_shared<SynRockAgent>(
            &robot->GetRocks(), chrono_data_path, /*visualize_zombies=*/false, rock_field_config,
            // The builder's feedstock rides the same agent rather than a new one, because
            // a new AddAgent would shift every agent id declared above.
            &builder_pile_rocks, BuilderWallSlotCount(num_robot_ranks)));

        // Skipped entirely under --no_builder, which is why that flag shifts the agent
        // ids of everything registered after it.
        if (builder) {
            syn_manager.AddAgent(chrono_types::make_shared<AmdTrackedVehicleAgent>(
                builder->GetVehicle(), amd_uw_data_path + "synchrono/vehicle/M113.json"));
            syn_manager.AddAgent(chrono_types::make_shared<SynArmAgent>(
                builder->GetArm() ? builder->GetArm()->GetBodies()
                                  : std::vector<std::shared_ptr<ChBodyAuxRef>>{},
                amd_uw_data_path, builder_arm_shapes_dir, builder_arm_geometry_scale,
                /*visualize_zombies=*/false));
        }

        syn_manager.AddAgent(chrono_types::make_shared<SynTrailerBedAgent>(
            robot->GetTrailerBed(), robot->GetTrailerTailgate(), /*visualize_zombies=*/false));

        // The rover's arm. Must stay LAST of this rank's AddAgent calls so the agent
        // ids declared above keep matching registration order.
        syn_manager.AddAgent(chrono_types::make_shared<SynArmAgent>(
            robot->GetArm() ? robot->GetArm()->GetBodies()
                            : std::vector<std::shared_ptr<ChBodyAuxRef>>{},
            amd_uw_data_path, rover_arm_shapes_dir, rover_arm_geometry_scale,
            /*visualize_zombies=*/false));

        if (builder) {
            const ChVector3d builder_pos = builder->GetPosition();
            SynLog() << "Rank " << rank << " builder at (" << builder_pos.x() << ", " << builder_pos.y() << ", "
                     << builder_pos.z() << ").\n";
        } else {
            SynLog() << "Rank " << rank << " has NO builder (--no_builder).\n";
        }
    } else {
        syn_manager.AddAgent(chrono_types::make_shared<SynEnvironmentAgent>(system));
        SynLog() << "Rank 0 is sensor/visualization only; robot physics starts on rank 1.\n";
    }

    for (int robot_rank = 1; robot_rank < num_ranks; robot_rank++) {
        if (robot_rank == rank)
            continue;

        // Only the rank that draws the whole site builds zombie bodies. Every other
        // rank chase-cams its own vehicle, so remote machines were ~150 bodies apiece
        // (142 of them M113 running gear) created and stepped purely off-camera.
        // Terrain, rings and the centre pad are local, not zombies, so they remain.
        if (!draws_zombies) {
            for (int agent_id = 1; agent_id <= last_agent_id; ++agent_id)
                syn_manager.AddZombie(chrono_types::make_shared<SynQuietAgent>(), AgentKey(robot_rank, agent_id));
            continue;
        }

        // A zombie owns no rocks of its own; it only receives poses.
        // A zombie owns no rocks of its own; it only receives poses. The capacity must
        // still match the sender's, or the tail of the message has nowhere to be drawn.
        syn_manager.AddZombie(chrono_types::make_shared<SynRockAgent>(
                                  nullptr, chrono_data_path, /*visualize_zombies=*/true, rock_field_config,
                                  nullptr, BuilderWallSlotCount(num_robot_ranks)),
                              AgentKey(robot_rank, rock_agent_id));
        syn_manager.AddZombie(
            chrono_types::make_shared<SynArmAgent>(
                std::vector<std::shared_ptr<ChBodyAuxRef>>{}, amd_uw_data_path, builder_arm_shapes_dir,
                builder_arm_geometry_scale, /*visualize_zombies=*/true),
            AgentKey(robot_rank, builder_arm_agent_id));
        syn_manager.AddZombie(
            chrono_types::make_shared<SynArmAgent>(
                std::vector<std::shared_ptr<ChBodyAuxRef>>{}, amd_uw_data_path, rover_arm_shapes_dir,
                rover_arm_geometry_scale, /*visualize_zombies=*/true),
            AgentKey(robot_rank, rover_arm_agent_id));

        // Register our builder zombie explicitly: the agent factory only runs when no
        // zombie exists for that key, and subclassing on the OWNING rank would do
        // nothing (a rank calls InitializeZombie only on its zombies). Vis files come
        // from M113.json, since a pre-registered zombie gets no description message.
        syn_manager.AddZombie(chrono_types::make_shared<AmdTrackedVehicleAgent>(
                                  nullptr, amd_uw_data_path + "synchrono/vehicle/M113.json"),
                              AgentKey(robot_rank, tracked_builder_agent_id));

        // Likewise the trailer. Vis files and wheel count set here for the same reason.
        auto trailer_zombie = chrono_types::make_shared<SynTrailerAgent>();
        trailer_zombie->SetZombieVisualizationFiles("LRV_Wagon/trailer_chassis.obj",
                                                    "LRV/meshes/Polaris_wheel.obj",
                                                    "LRV/meshes/LRVtire_red_m.obj");
        trailer_zombie->SetNumWheels(2);
        syn_manager.AddZombie(trailer_zombie, AgentKey(robot_rank, trailer_agent_id));

        // The dump bed and tailgate, which no stock agent carries.
        syn_manager.AddZombie(
            chrono_types::make_shared<SynTrailerBedAgent>(nullptr, nullptr, /*visualize_zombies=*/true),
            AgentKey(robot_rank, trailer_bed_agent_id));
    }

    // The SolidWorks importer's embedded `import pychrono` (in LrvArm) resets
    // Chrono's global data paths to the library's compiled-in default. That breaks
    // the zombie vehicle mesh lookups in Initialize() below (they resolve
    // "vehicle/LRV/meshes/..." against the wrong root and segfault on the missing
    // OBJ). Re-assert our data paths right before zombie creation so the lookups
    // resolve against the AMD-UW vehicle data. No-op for the non-importer build.
    SetChronoDataPath(vehicle_data_path);
    SetVehicleDataPath(vehicle_data_path);

    const size_t bodies_before_zombies = system->GetBodies().size();
    syn_manager.Initialize(system);
    // Zombie creation happens inside Initialize. Catches a rank silently going back
    // to building other ranks' bodies.
    std::cout << "[main] rank " << rank << ": " << bodies_before_zombies << " own bodies + "
              << (system->GetBodies().size() - bodies_before_zombies) << " zombie bodies = "
              << system->GetBodies().size() << " total\n";

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
        if (sensor_frame_dir.empty()) {
            global_camera->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
                global_camera_width, global_camera_height, "Global Camera"));
        } else {
            // Saving instead of showing, so this works with no display attached.
            global_camera->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_frame_dir));
            std::cout << "[main] global camera frames -> " << sensor_frame_dir << "\n";
        }
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
        // Reuses --sensor_frame_dir: one flag captures whichever view this rank has.
        if (!sensor_frame_dir.empty())
            app.SaveFrames(sensor_frame_dir, 40);
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
                // The builder's station is its OWN, driven by how much wall it has laid
                // and by nothing else.
                //
                // It used to be RankRayAngleRad(..., robot->GetHarvestCycle()) -- the
                // builder was parked on whatever bearing its collector's lane happened to
                // be on, and it only ever moved when that collector finished a load and
                // dumped. So a builder was idle for an entire harvest (minutes of sim),
                // then teleported 30 degrees round the site in one step of the station
                // angle, and its arm had nothing to do at either end. Now it steps one
                // wall slot -- 0.9 m of course, 0.99 m of lane -- each time it lays a
                // rock, which is a thing it does entirely on its own.
                //
                // One refinement on top of that: the station is pulled HALF WAY toward the
                // rock actually on offer. Nominally the two agree -- HarvestDropSlot puts
                // each load on the slot the builder will be at -- but a load is tipped out
                // of a moving trailer, so where it lands has a metre or two of scatter in
                // it. Splitting the difference keeps the wall slot (3.1 m inboard) and the
                // rock (~3.9 m outboard) both inside the arm's 5.2 m envelope for an
                // offset either way, instead of the builder parking exactly on its slot and
                // staring at a pile it cannot quite reach. Clamped, so a stray rock across
                // the site can never drag a builder off its course.
                //
                // The station is the SLOT's station and nothing else. It is a function of
                // one monotonically increasing integer, so it can only ever advance --
                // which is the invariant the drive law depends on, because the builder
                // orbits one way and a station that retreats by any amount costs it a full
                // lap (207 m at 0.9 m/s, ~230 s of sim) to reach a point it was already
                // standing on.
                //
                // It briefly also carried a term pulling it half way toward whichever rock
                // was on offer, as insurance against a tipped load landing off its nominal
                // slot. That was unnecessary and it was harmful. Unnecessary: the audit
                // puts a delivered pile 3.91-4.04 m from the arm base against a 5.0 m
                // limit, so the envelope already absorbs sqrt(5.0^2 - 3.9^2) = 3.1 m of
                // arc, about three slots of scatter, with the hull parked exactly on its
                // slot. Harmful: the pull collapses to zero the moment that rock leaves
                // reach, and a collapsing pull is a retreating station -- measured on
                // builder 3, which held at 186.1 deg and then stepped back to 184.3 deg,
                // buying itself a lap. Latching the maximum fixed the retreat but left the
                // station ratcheted up to a slot ahead of its own wall slot, which is the
                // wrong pose to place from. Simplest correct answer: no pull.
                if (builder) {
                    const int idx = robot->GetRobotIndex();
                    const double slot_angle = RankRayAngleRad(idx, num_robot_ranks) +
                                              builder->GetPlacedCount() * wall_slot_pitch_rad;
                    builder->SetStationAngle(slot_angle + BuilderArmLeadRad());
                    builder->Synchronize(time);
                }
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
                if (builder)
                    builder->Advance(step_size);
            }
            {
                ScopedTimer timer(perf_accum.robot_adv);
                robot->Advance(step_size);
                app.Advance(step_size);
            }
            robot->LogMotionIfNeeded(step_number, motion_log_steps, terrain);

            // A rank whose physics has gone non-finite is not going to recover, and
            // every rank is in MPI lockstep, so letting it run wastes the whole job:
            // in the last 3 h run one rank died 1 h 39 m in and the other three kept
            // simulating around a corpse for 90 minutes. It also poisons rank 0,
            // which keeps receiving NaN zombie poses over SynChrono. Take the whole
            // job down with a message that says which rank and when. RobotRig has
            // already printed the body and the constraint reactions that named it.
            if (robot->HasDiverged()) {
                std::cout << "[main] rank " << rank << " has diverged at t=" << time
                          << "; aborting the job. See the [RobotRig] DIVERGING/DIVERGED report above for the "
                             "body and the link reactions that caused it.\n";
                std::cout.flush();
                MPI_Abort(MPI_COMM_WORLD, 1);
            }
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
                     << " z=" << builder_pos.z() << " shoe_dmax=" << builder->GetMaxShoeDistance()
                     << " parked=" << (builder->IsParked() ? 1 : 0)
                     // Drift from where this builder was placed. The orbit controller only
                     // closes the loop on orbit ANGLE, so radial drift shows up here and
                     // nowhere else.
                     << " drift=" << (builder_pos - builder_spawn_pos).Length() << "\n";
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
