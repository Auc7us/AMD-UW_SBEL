#include "BuilderRig.h"

#include "RobotLayout.h"

#include <algorithm>
#include <stdexcept>
#include <string>

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChTypes.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"
#include "chrono_models/vehicle/m113/M113.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "LrvArm.h"
#include "MaterialUtils.h"
#ifdef AMD_UW_ENABLE_ROS2
#include "BuilderArmRosBridge.h"
#include "BuilderVehicleRosBridge.h"
#endif

namespace amd_uw {

namespace {

void AddSquashedChassisVisual(const std::shared_ptr<chrono::ChBody>& chassis_body,
                              const std::string& amd_uw_data_path,
                              const chrono::ChColor& color) {
    // The stock M113 hull roof is far above the arm mount and visually buries
    // most of the arm. Preserve the hull's full X length and Y width, but
    // compress only its vertical (Z) mesh coordinates until its highest vertex
    // touches the underside of the arm's scaled base at the existing mount.
    //
    // Exported mesh/model measurements in the chassis reference frame:
    //   stock M113 Chassis.obj top                 z = 2.247088 m
    //   arm mount                                  z = 0.400000 m
    //   unscaled arm base mesh top before its flip z = 0.076200 m
    //   arm geometry scale                             2.0
    //
    // The base's 180-degree Y rotation maps that mesh top to its underside:
    //   arm underside z = 0.4 - 2 * 0.0762 = 0.2476 m.
    constexpr double chassis_mesh_top_z = 2.247088;
    constexpr double arm_mount_z = 0.4;
    constexpr double arm_geometry_scale = 2.0;
    constexpr double arm_base_mesh_top_z = 0.0762;
    constexpr double arm_underside_z =
        arm_mount_z - arm_geometry_scale * arm_base_mesh_top_z;
    constexpr double chassis_z_scale = arm_underside_z / chassis_mesh_top_z;

    auto mesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(
        amd_uw_data_path + "vehicle/M113/meshes/Chassis.obj", true, true);
    if (!mesh)
        throw std::runtime_error("Cannot load builder chassis mesh.");

    mesh->Transform(
        chrono::VNULL,
        chrono::ChMatrix33<>(chrono::ChVector3d(1.0, 1.0, chassis_z_scale)));

    auto visual = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    visual->SetMesh(mesh);
    visual->SetName("Builder_Chassis_Squashed_Z");
    visual->SetMutable(false);
    // Same colour as this rank's collector trailer bed, so the pair reads as one
    // team. The mesh has no material of its own, which is why every builder came
    // out the default white.
    visual->SetColor(color);
    chassis_body->AddVisualShape(visual);
}

}  // namespace

BuilderRig::BuilderRig(int rank,
                       chrono::ChSystem* system,
                       const std::string& amd_uw_data_path,
                       const chrono::ChCoordsys<>& init_pose,
                       const Options& options) {
    using namespace chrono;
    using namespace chrono::vehicle;

    // Shared-system M113 constructor: the builder joins this rank's existing
    // world so it can contact that rank's rocks and terrain. Contact method and
    // collision system come from the system that already exists, so the
    // corresponding setters are deliberately not called here.
    m_m113 = std::make_unique<chrono::vehicle::m113::M113>(system);
    m_m113->SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    m_m113->SetDoublePinTrackShoeType(DoublePinTrackShoeType::ONE_CONNECTOR);
    m_m113->SetTrackBushings(false);
    m_m113->SetSuspensionBushings(false);
    m_m113->SetTrackStiffness(false);
    m_m113->SetDrivelineType(DrivelineTypeTV::BDS);
    m_m113->SetBrakeType(BrakeType::SHAFTS);
    m_m113->SetEngineType(EngineModelType::SIMPLE);
    m_m113->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    m_m113->SetChassisCollisionType(CollisionType::NONE);

    // The chassis must NOT be fixed, even though the builder starts parked.
    //
    // The Python reference parks it by pinning the hull, which is self-consistent
    // on its flat terrain: a level hull at the ride height puts every road wheel
    // at the same height. On this heightmap the hull is placed level (yaw only)
    // over ground that pitches and rolls a few degrees, with 0.24-0.33 m of height
    // spread across the hull footprint. Pinning the hull leaves the suspension no
    // way to absorb that, so one end of the track is driven into the ground, the
    // SMC penalty force has nowhere to go, and the single-pin chain throws shoes
    // and goes NaN within about a second -- on every rank, whatever the local
    // slope. It only ever looked healthy because the orbit controller released the
    // hull within ~50 ms of the run starting.
    //
    // A free hull with the brakes on settles onto the real surface instead, which
    // is what the rover already does (see RobotRig::Settle). The brakes hold it:
    // mu=0.9 on locked tracks easily statically holds these slopes.
    m_m113->SetChassisFixed(false);
    m_m113->CreateTrack(true);
    m_m113->SetInitPosition(init_pose);
    m_m113->Initialize();

    // Install a builder-specific, vertically squashed hull visual below. This
    // leaves the physical chassis and every tracked-running-gear visual intact.
    m_m113->SetChassisVisualizationType(VisualizationType::NONE);
    m_m113->SetSprocketVisualizationType(VisualizationType::MESH);
    m_m113->SetIdlerVisualizationType(VisualizationType::MESH);
    m_m113->SetSuspensionVisualizationType(VisualizationType::MESH);
    m_m113->SetIdlerWheelVisualizationType(VisualizationType::MESH);
    m_m113->SetRoadWheelVisualizationType(VisualizationType::MESH);
    m_m113->SetTrackShoeVisualizationType(VisualizationType::MESH);

    // Gravity, solver, and timestepper belong to the shared system and are set
    // once by the caller (main.cpp) -- the builder must not reconfigure a world
    // it does not own.

    // No terrain is created here: the builder drives the rank's existing terrain,
    // which is also what its rocks rest on.
    const auto chassis_body = m_m113->GetChassisBody();
    AddSquashedChassisVisual(chassis_body, amd_uw_data_path, RankColor(rank - 1));
    const ChQuaternion<> chassis_rot = chassis_body->GetRot();
    // Exact TrackedVeh_Builder.py reference mount and scale. The reference arm
    // uses a 2x geometric scale about this chassis-frame mount while preserving
    // 1x finger geometry and mass/inertia values.
    constexpr double arm_geometry_scale = 2.0;
    const ChVector3d arm_offset(-2.5, 0.0, 0.4);
    const ChVector3d arm_pos = m_m113->GetChassis()->GetPos() + chassis_rot.Rotate(arm_offset);
    const ChQuaternion<> arm_rot = chassis_rot * QuatFromAngleZ(CH_PI);

    // Build from the dedicated M113 arm copy without starting a second embedded
    // Python interpreter in this MPI process (the rover arm already started
    // one). The native path uses the same exported body poses, meshes,
    // masses/inertias, mount frames, finger contacts, and motorized joints.
    if (options.with_arm) {
        m_arm = std::make_unique<LrvArm>(
            system, chassis_body, amd_uw_data_path, arm_pos, arm_rot,
            /*import_solidworks=*/false, "builder_" + std::to_string(rank) + "_",
            "m113_builder_arm/m113_builder_arm.py",
            "m113_builder_arm/m113_builder_arm_shapes/",
            /*parked_rigid=*/false,
            arm_geometry_scale);
    }

#ifdef AMD_UW_ENABLE_ROS2
    if (m_arm)
        m_arm_ros_bridge = std::make_unique<BuilderArmRosBridge>(rank, *m_arm);
    m_vehicle_ros_bridge =
        std::make_unique<BuilderVehicleRosBridge>(rank, m_m113->GetVehicle());
#endif

    // The rover's arm import runs the SolidWorks parser, which leaves the shared
    // collision system in a state where bodies added afterwards never register
    // contacts (see RobotRig::InitializeOnTerrain). The builder and its arm are
    // added after that, so rebind everything or the tracks fall through the
    // ground and the gripper passes through rocks.
    system->GetCollisionSystem()->BindAll();

    m_driver_inputs.m_steering = 0.0;
    m_driver_inputs.m_throttle = 0.0;
    m_driver_inputs.m_braking = 1.0;
}

BuilderRig::~BuilderRig() = default;

chrono::ChSystem* BuilderRig::GetSystem() const {
    return m_m113->GetSystem();
}

chrono::vehicle::ChTrackedVehicle* BuilderRig::GetVehicle() const {
    return &m_m113->GetVehicle();
}

LrvArm* BuilderRig::GetArm() const {
    return m_arm.get();
}

chrono::ChVector3d BuilderRig::GetPosition() const {
    return m_m113->GetChassis()->GetPos();
}

double BuilderRig::GetSpeed() const {
    return m_m113->GetVehicle().GetSpeed();
}

double BuilderRig::GetMaxShoeDistance() const {
    using namespace chrono::vehicle;
    const auto& vehicle = m_m113->GetVehicle();
    const chrono::ChVector3d chassis_pos = vehicle.GetChassis()->GetPos();
    double max_distance = 0.0;
    for (auto side : {LEFT, RIGHT}) {
        const auto track = vehicle.GetTrackAssembly(side);
        for (size_t i = 0; i < track->GetNumTrackShoes(); ++i) {
            const double distance = (track->GetTrackShoePos(i) - chassis_pos).Length();
            max_distance = std::max(max_distance, distance);
        }
    }
    return max_distance;
}

void BuilderRig::SetDriverInputs(const chrono::vehicle::DriverInputs& inputs) {
    m_driver_inputs = inputs;
}

void BuilderRig::Synchronize(double time) {
#ifdef AMD_UW_ENABLE_ROS2
    // The builder starts braked and simply holds until ROS takes over; there is no
    // hull to release, so a command is just a command.
    if (const auto command = m_vehicle_ros_bridge->Synchronize())
        m_driver_inputs = *command;
    if (m_arm_ros_bridge)
        m_arm_ros_bridge->Synchronize();
#endif
    if (m_arm)
        m_arm->Update(time);
    // No terrain argument: a tracked vehicle interacts with rigid terrain purely
    // through contacts, and the rank's terrain is synchronized by its owner.
    m_m113->Synchronize(time, m_driver_inputs);
}

void BuilderRig::SetStationAngle(double angle_rad) {
#ifdef AMD_UW_ENABLE_ROS2
    if (m_vehicle_ros_bridge)
        m_vehicle_ros_bridge->SetStationAngle(angle_rad);
#else
    (void)angle_rad;
#endif
}

void BuilderRig::Advance(double step) {
    // Subsystems only. The shared system's DoStepDynamics is issued once, by the
    // rig that owns the system.
    m_m113->Advance(step);
}

}  // namespace amd_uw
