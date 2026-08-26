#include "BuilderRig.h"

#include "RobotLayout.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

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

    // Do NOT rescale the idler tensioner for lunar gravity. It looks wrong on paper --
    // the stock 2e4 N preload and 1e6 N/m spring are Earth numbers, and at 1.62 the
    // tensioners push with more than the vehicle's whole weight -- but it was measured:
    // scaling all three coefficients by g/9.81 moved this rank's divergence from clean
    // to t=11.30, and combined with a 33 m builder lane from t=13.35 to t=6.44. The
    // softer spring lets the chain go slack, which is worse than the hard preload.

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
        m_arm_ros_bridge =
            std::make_unique<BuilderArmRosBridge>(rank, *m_arm, options.seed_rocks, options.wall_slots);
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
    // The builder holds the brake until ROS takes over -- see the initialiser on
    // m_driver_inputs, which is what actually makes that true. Retaining the last
    // command between messages is deliberate: the controller runs at 20 Hz and the sim
    // steps at 2 kHz.
    if (time < m_command_enable_time) {
        // Settling window. The bridge is still spun so its subscription queue cannot
        // back up, but whatever the controller sent is discarded and the brake stays on.
        m_vehicle_ros_bridge->Synchronize(time);
        m_driver_inputs = chrono::vehicle::DriverInputs{0.0, 0.0, 1.0, 0.0};
    } else if (const auto command = m_vehicle_ros_bridge->Synchronize(time)) {
        m_driver_inputs = *command;
    }
#endif
    // DO NOT PIN THE HULL. m_parked is a STATE, not an action: it means "the controller
    // is holding the brake and the hull has actually stopped", which is the condition the
    // arm bridge needs before it solves a pose against the arm base frame. Nothing here
    // calls SetFixed, and it must not.
    //
    // Pinning was tried and measured, twice, and it tears the single-pin track apart.
    // SetFixed does not decelerate a body, it removes it from the solve, so the chassis
    // velocity is forced to zero in one step while ~130 track shoes, the road wheels and
    // the idler tensioners are still solving against the velocity it had a step ago. Same
    // signature every time: idler carriers at 140-364 rad/s and 25 m/s within ~0.6 s of
    // the pin. Bisected at 3 ranks with no controllers running:
    //
    //   pin on brake, ungated       dead at t=2.115  (pinned at the settle window, t=1.5)
    //   pin only when |v| < 0.02    dead at t=6.085  (gate delays it; does not fix it)
    //   --no_hull_park              clean past t=21
    //
    // So the gate was not the answer -- the act of pinning is. An earlier comment here
    // claimed plane seating had made pinning safe; the middle row above is what disproves
    // it, because that hull was both plane-seated AND stationary when it was pinned.
    //
    // The creep that pinning was introduced to stop is now largely harmless anyway: the
    // arm solves every target against the LIVE arm_base_pose rather than a nominal
    // station, and the wall slots are fixed world points, so a builder a little off its
    // mark still lays a straight course. Along-arc creep is corrected actively by the
    // orbit controller's station keeping (0.12 m deadband). What remains uncorrected is
    // radial and yaw drift, and the 2.78-4.44 m reach band absorbs a good deal of it --
    // nominal reaches are 3.09 m to the wall and 3.5 m to the heap.
    //
    // A braked M113 does not stay put. The reference scenario measured this directly --
    // 0.08 m / 0.6 deg of drift at heading 0, but up to 0.9 m / 17 deg at 45/90/135 deg
    // over 8 s, and it persists with the brakes RELEASED, so it is not a brake problem.
    // Ours sits at heading = ray + 90 deg, i.e. one of the bad ones. Along-arc drift is
    // corrected by the orbit controller's station keeping; RADIAL drift is not, because
    // station_error is an orbit angle and cannot see it. That is a known, accepted
    // residual -- rigid terrain gives a locked track nothing to key into, and the plan is
    // to move to SCM, where a deformable surface should hold it.
    //
    // Full brake with zero throttle IS the park signal -- it is exactly what the orbit
    // controller publishes when it is on station, so no extra topic is needed.
    //
    // The speed test is the other half. "Parked" has to mean the arm base frame is not
    // moving, because that frame is what every solved pose is expressed in; a hull that is
    // braked but still rolling to a stop would have the arm reaching for where the rock
    // was a moment ago.
    if (time >= m_command_enable_time) {
        constexpr double park_speed_tol = 0.02;  // m/s
        constexpr double park_spin_tol = 0.02;   // rad/s
        const auto chassis = m_m113->GetChassisBody();
        const bool wants_park = m_hull_park_enabled && m_driver_inputs.m_throttle <= 0.0 &&
                                m_driver_inputs.m_braking >= 1.0;

        // VIRTUAL ANCHOR. Brakes alone do not hold this vehicle: measured on full brake,
        // zero throttle, on station, the hull still creeps at 0.22-0.27 m/s. So the arm
        // was never offered a pick, because "parked" means "the arm base frame is steady"
        // and it never was.
        //
        // SetFixed does hold it, and was tried, and tears the single-pin track apart --
        // bisected at 3 ranks: pinned on brake dead at t=2.115, pinned only when already
        // stopped dead at t=6.085, never pinned clean past t=21. SetFixed removes the body
        // from the solve, so its velocity is forced to zero in one step while ~130 track
        // shoes are still solving against the velocity it had a step ago.
        //
        // A spring-damper to the parked pose has no such discontinuity: it is an ordinary
        // external load, it ramps, and the track sees a force it can react to. Stiffness
        // is sized from what it must resist -- the downslope component of a 10483 kg hull
        // at lunar gravity on the ~3.5 deg local tilt is about 1.0 kN, so 5e4 N/m reaches
        // that at 2 cm of drift. Damping is near-critical for that stiffness and mass.
        if (wants_park && !m_anchor_active) {
            m_anchor_active = true;
            m_anchor_pos = chassis->GetPos();
            m_anchor_yaw = chassis->GetRot().GetCardanAnglesZYX().z();
            if (!m_anchor_accumulator_ready) {
                m_anchor_accumulator = chassis->AddAccumulator();
                m_anchor_accumulator_ready = true;
            }
        } else if (!wants_park && m_anchor_active) {
            m_anchor_active = false;
            if (m_anchor_accumulator_ready)
                chassis->EmptyAccumulator(m_anchor_accumulator);
        }

        if (m_anchor_active && m_anchor_accumulator_ready) {
            constexpr double anchor_k = 5.0e4;        // N/m
            constexpr double anchor_c = 4.5e4;        // N.s/m, near-critical at 10.5 t
            constexpr double anchor_k_yaw = 2.0e5;    // N.m/rad
            constexpr double anchor_c_yaw = 1.2e5;    // N.m.s/rad

            // The anchor HOLDS the parked pose. It does not correct it, and it cannot.
            //
            // It was briefly made to walk its setpoint radially back onto the lane, to
            // undo the inward drift the builder accumulates while creeping between slots.
            // That cannot work, and the arithmetic says so without needing a run: sliding
            // a braked tracked hull sideways has to beat track-ground friction, which at
            // mu = 0.9 on a 10483 kg machine in lunar gravity is 0.9 * 10483 * 1.62 =
            // 15.3 kN. The anchor's ceiling is k * max_offset = 5e4 * 0.15 = 7.5 kN, under
            // half of it. It was sized against the gravity component on a 5 deg slope
            // (1.5 kN), which is the force it has to RESIST, not the one it would have to
            // OVERCOME. Raising it past 15 kN is not the answer either: that is a large
            // force fed into a braked single-pin track, which is how pinning destroyed the
            // chain in the first place.
            //
            // So radial error is the drive controller's problem, corrected by driving --
            // and its release band is sized to react while the arm can still work and long
            // before the hull reaches the wall it laid. See station_radius_release_m.
            //
            // Horizontal only: the vertical direction is the suspension's job, and pulling
            // on it would fight the track's contact with the ground.
            const chrono::ChVector3d offset = chassis->GetPos() - m_anchor_pos;
            const chrono::ChVector3d vel = chassis->GetPosDt();
            const chrono::ChVector3d force(-anchor_k * offset.x() - anchor_c * vel.x(),
                                           -anchor_k * offset.y() - anchor_c * vel.y(), 0.0);
            const double yaw = chassis->GetRot().GetCardanAnglesZYX().z();
            double yaw_err = yaw - m_anchor_yaw;
            while (yaw_err > chrono::CH_PI)
                yaw_err -= chrono::CH_2PI;
            while (yaw_err < -chrono::CH_PI)
                yaw_err += chrono::CH_2PI;
            const double torque_z =
                -anchor_k_yaw * yaw_err - anchor_c_yaw * chassis->GetAngVelParent().z();

            chassis->EmptyAccumulator(m_anchor_accumulator);
            chassis->AccumulateForce(m_anchor_accumulator, force, chassis->GetPos(), false);
            chassis->AccumulateTorque(m_anchor_accumulator,
                                      chrono::ChVector3d(0.0, 0.0, torque_z), false);
        }

        m_parked = wants_park && chassis->GetPosDt().Length() < park_speed_tol &&
                   chassis->GetAngVelParent().Length() < park_spin_tol;
    }

#ifdef AMD_UW_ENABLE_ROS2
    // AFTER the park decision, because the arm bridge will not offer or accept a pick
    // unless the hull is pinned: a solved grab pose is expressed in the arm base frame,
    // and that frame is moving whenever the builder is creeping to its next station.
    //
    // The arm is gated by the same settle window as the drive. It was previously
    // synchronized unconditionally, so the arm began slewing its first ~4.7 rad swing at
    // t=0 while the hull was still settling -- the one thing the window exists to prevent.
    if (m_arm_ros_bridge) {
        m_arm_ros_bridge->SetHullParked(m_parked);
        m_arm_ros_bridge->Synchronize(time, /*apply_commands=*/time >= m_command_enable_time);
    }
#endif

    // The single Update for this step. The arm bridge deliberately does not call it --
    // two calls in one step would advance the joint slew twice.
    if (m_arm)
        m_arm->Update(time);
    // No terrain argument: a tracked vehicle interacts with rigid terrain purely
    // through contacts, and the rank's terrain is synchronized by its owner.
    m_m113->Synchronize(time, m_driver_inputs);
}

int BuilderRig::GetPlacedCount() const {
#ifdef AMD_UW_ENABLE_ROS2
    return m_arm_ros_bridge ? m_arm_ros_bridge->GetPlacedCount() : 0;
#else
    return 0;
#endif
}

void BuilderRig::SetDeliveredRockSource(
    std::function<std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>()> source) {
#ifdef AMD_UW_ENABLE_ROS2
    if (m_arm_ros_bridge)
        m_arm_ros_bridge->SetDeliveredRockSource(std::move(source));
#else
    (void)source;
#endif
}

void BuilderRig::SetStationAngle(double angle_rad) {
#ifdef AMD_UW_ENABLE_ROS2
    // The arm bridge gets to slide the station along its lane when it is starved and a
    // rock is lying just outside its envelope. Added here rather than in main.cpp so the
    // caller keeps computing the slot's own angle and nothing else has to know about it.
    // See BuilderArmRosBridge::GetStationFetchOffset.
    if (m_arm_ros_bridge)
        angle_rad += m_arm_ros_bridge->GetStationFetchOffset();
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
