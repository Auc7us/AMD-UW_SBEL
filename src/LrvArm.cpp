#include "LrvArm.h"

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChTypes.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChContactMaterial.h"
#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
#include "chrono_parsers/ChParserPython.h"
#endif

#include "MaterialUtils.h"
#include "RobotLayout.h"

namespace amd_uw {

namespace {

constexpr double approach_2_time = 1.0;
constexpr double close_timeout = 8.0;
// APPROACH settle gating: judge the pose only once the gripper has actually
// arrived and quieted, then sanity-check the reach.
constexpr double approach_min_settle = 0.4;  // min time at a pose before judging it
constexpr double settle_speed_tol = 0.05;    // gripper speed (m/s) below which it's "settled"
constexpr double divergence_abort = 1.0;     // gripper this far off the target => bad pose, fail
constexpr double control_dt = 0.01;
constexpr double finger_open_sep = 0.388;
constexpr double finger_close_pos = 0.145;
constexpr double finger_grasp_sep = 0.26;
constexpr double finger_close_speed = 0.1;
constexpr double lock_finger_dist = 0.27;
constexpr double grip_force_tol = 30.0;
constexpr double lift_theta2 = chrono::CH_PI / 3.0;
constexpr double lift_speed = 0.5;
constexpr double lift_delay = 1.5;
constexpr double place_tol = 0.15;
constexpr double place_min_settle = 0.4;
constexpr double place_timeout = 6.0;
constexpr double release_hold_time = 1.0;
constexpr double stow_hold_time = 2.0;
constexpr double total_timeout = 45.0;
// Max rate any joint's COMMANDED angle may change, rad/s. The largest pose change
// in the sequence is roughly pi rad (rest/stow to a grab pose), so ~2 s to cross it
// -- comfortably inside close_timeout (8 s) and place_timeout (6 s), and slower
// than LIFTING's own 0.5 rad/s ramp so that phase is unaffected.
// Reach envelope for an accepted grab target, measured in the arm-base frame --
// the same frame the Python solver works in (world_to_arm_base_local there is
// GetIkFrameRot().RotateBack(target - GetIkFramePos()) here).
//
// The Python IK is an UNCONSTRAINED BFGS minimisation of the FK residual: no joint
// limits, no reach limits. So it will happily return a pose for a point underneath
// the arm's own base by folding the arm back through the rover, and report
// fk_err = 0.0000 -- which only ever meant "this pose is self-consistent", never
// "this pose is legal". Nothing downstream checked either, and the six actuators
// are CONSTRAINT motors (ChLinkMotorRotationAngle / ChLinkMotorLinearPosition):
// they have no torque limit, so the solver drives the collision-enabled fingers
// into the chassis with whatever force it takes, and that reaction goes straight
// into the ChLinkLockLock welding the arm base to the chassis.
//
// Observed in the 3 h run: rank 4 was handed local_target = (-0.177, -0.324,
// -0.541), i.e. the gripper commanded to a point 0.37 m from its own base, behind
// it and half a metre down. Every one of the 30 legitimate grabs in that run sat
// between 1.39 m and 2.22 m of horizontal reach, so the two populations do not
// overlap and this bound does not cost a single good grab.
//
// The arm spans ~2.72 m from base origin to fingertip at geometry_scale 1, so the
// upper bound is just inside full extension: a target beyond it can only be
// reached by a solution that is straight-armed and singular.
//
// All three are LENGTHS, so they scale with the arm. The builder's arm is the same
// model at geometry_scale 2.0: its links are twice as long, it spans 5.44 m instead of
// 2.72 m, and it works a heap 3.5 m from its base. Left unscaled these bounds reject
// every legitimate builder grab (3.5 > 2.6) with error_code 5 -- the arm would refuse
// every rock it was parked next to. divergence_abort is the same kind of quantity (a
// gripper position error) and scales for the same reason. The FINGER constants below do
// NOT scale: finger geometry stays 1x on the builder (see BuilderRig).
constexpr double min_grab_reach_xy = 1.0;
constexpr double max_grab_reach_xy = 2.6;
constexpr double min_grab_local_z = -1.2;
constexpr double joint_slew_rate = 1.5;
// Same, for the finger prismatic motors, m/s. Opening used to be a single step
// from finger_close_pos to 0 -- 0.145 m demanded in one 5e-4 s step, a commanded
// 290 m/s -- fired at the instant the rock is released over the bed. Closing was
// always ramped (finger_close_speed); only the release snapped. 0.3 m/s clears the
// full travel in ~0.5 s, inside release_hold_time.
constexpr double finger_slew_rate = 0.3;
constexpr std::array<double, 4> stow_theta = {-chrono::CH_PI, chrono::CH_PI / 5.0, chrono::CH_PI / 4.0, 0.0};

struct BodySpec {
    const char* name;
    const char* mesh;
    chrono::ChVector3d pos;
    chrono::ChQuaternion<> rot;
    double mass;
    chrono::ChVector3d inertia_xx;
    chrono::ChVector3d inertia_xy;
    chrono::ChVector3d com;
    bool finger = false;
};

struct FrameSpec {
    chrono::ChVector3d pos;
    chrono::ChQuaternion<> rot;
};

const std::array<BodySpec, 8> arm_bodies = {{
    {"endeffector-1", "body_1_1.obj", {-2.667, -8.70558992690816e-15, 0.325155513123522},
     {0.707106781186548, -1.20712009714022e-16, -0.707106781186547, 1.99216632648211e-16},
     0.626524227569578, {0.0026237186559956, 0.000502977063197131, 0.00238484258461985},
     {-9.6347345014623e-19, 9.62218303335903e-21, 2.8538491491407e-19},
     {7.02339112983057e-18, -1.20943926376326e-17, 0.023329601404936}},
    {"bicep-1", "body_2_1.obj", {-8.07010409222406e-17, 1.50119809845186e-16, 0.325155513123522},
     {-0.5, 0.5, -0.5, -0.5}, 10.0188345607749,
     {1.85127451103256, 1.82217210560919, 0.0315263121628376},
     {-1.11658003282484e-16, -1.25599415755622e-15, -3.04765594934145e-12},
     {-1.8504257992554e-14, 0.559895724862395, -2.67581948957994e-17}},
    {"base-1", "body_3_1.obj", {-3.75783253987686e-62, -2.15904213877361e-78, 0.0762000000000001},
     {0, -2.77555756156289e-17, 1, 0}, 8.00691853432187,
     {0.180051635759179, 0.180051635759179, 0.35121838172002},
     {-9.44741569709806e-17, -5.5851666924261e-18, 3.68087073319269e-18},
     {-6.34810319695294e-17, -2.61923103410295e-17, 0.0383731639052751}},
    {"shoulder-1", "body_4_1.obj", {-5.74189588383473e-19, 7.96390791413929e-18, 0.127},
     {-2.05721257448836e-17, -0.707106781186548, 0.707106781186548, -2.37690812695503e-17},
     17.3090829245461, {0.329102703458248, 0.359738630712971, 0.19573561096493},
     {-3.87519269175668e-08, -0.000888607344128861, 1.45843140654102e-08},
     {1.41950080889034e-09, -0.000931439058273036, -0.0142042923840082}},
    {"elbow-1", "body_5_1.obj", {-1.27, -8.49769687811631e-17, 0.325155513123522},
     {1.54074395550979e-33, 2.77555756156289e-17, 3.08148791101958e-33, 1}, 14.504670859222,
     {0.04629602603643, 2.96037800858872, 2.94728758334825},
     {6.40049612112032e-10, 4.61982454463747e-08, 2.19159727148393e-09},
     {0.571499998917816, 1.58835288481593e-09, 5.43151898041859e-08}},
    {"wrist-1", "body_6_1.obj", {-2.413, -8.23262472967057e-16, 0.325155513123523},
     {-9.81307786677359e-17, -9.81307786677358e-17, 0.707106781186548, 0.707106781186547},
     1.49908324319661, {0.00191172422781849, 0.0108644156852781, 0.00979181706223904},
     {2.61753783284415e-19, -2.6367345673623e-18, -1.35562279998041e-19},
     {0.10734177092476, -2.26013585595982e-18, 4.96127502994239e-18}},
    {"finger-2", "body_7_1.obj", {-2.7178, -0.101600000000009, 0.325155513123521},
     {0.707106781186547, -4.35788199605262e-32, 3.92523114670944e-17, 0.707106781186548},
     0.204687843355227, {0.000246140040298663, 0.000981000417461611, 0.0012051310562571},
     {-0.000226453080722204, -1.40752695675067e-19, -7.90586859128437e-21},
     {-0.0923087540409708, 0.0524076781696285, -2.18495338472112e-19}, true},
    {"finger-1", "body_7_1.obj", {-2.7178, 0.101599999999991, 0.325155513123523},
     {9.13822863364739e-34, 0.707106781186548, -0.707106781186547, 3.92523114670944e-17},
     0.204687843355227, {0.000246140040298663, 0.00098100041746161, 0.0012051310562571},
     {0.000226453080722204, -1.40752695675066e-19, 7.90586859128436e-21},
     {-0.0923087540409708, 0.0524076781696285, -2.18495338472112e-19}, true},
}};

chrono::ChFramed TransformFrame(const FrameSpec& frame,
                                const chrono::ChVector3d& mount_pos,
                                const chrono::ChQuaternion<>& mount_rot,
                                double geometry_scale) {
    return chrono::ChFramed(mount_pos + mount_rot.Rotate(frame.pos * geometry_scale), mount_rot * frame.rot);
}

std::shared_ptr<chrono::ChBodyAuxRef> CreateBody(chrono::ChSystem* system,
                                                 const BodySpec& spec,
                                                 const std::string& shapes_dir,
                                                 const chrono::ChVector3d& mount_pos,
                                                 const chrono::ChQuaternion<>& mount_rot,
                                                 const std::string& name_prefix,
                                                 double geometry_scale) {
    auto body = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    body->SetName(name_prefix + spec.name);
    body->SetMass(spec.mass);
    body->SetInertiaXX(spec.inertia_xx);
    body->SetInertiaXY(spec.inertia_xy);
    const chrono::ChVector3d com = spec.finger ? spec.com : spec.com * geometry_scale;
    body->SetFrameCOMToRef(chrono::ChFramed(com, chrono::QUNIT));

    // Match arm_model.py::_apply_scale exactly. Link bodies and their reference
    // positions scale about the model origin. The two fingers stay 1x but their
    // midpoint is carried to the scaled gripper tip.
    const chrono::ChVector3d finger_mid =
        0.5 * (arm_bodies[6].pos + arm_bodies[7].pos);
    chrono::ChVector3d model_pos = spec.pos;
    if (spec.finger)
        model_pos += finger_mid * (geometry_scale - 1.0);
    else
        model_pos *= geometry_scale;
    const chrono::ChVector3d ref_pos =
        spec.name == std::string("base-1") ? mount_pos : mount_pos + mount_rot.Rotate(model_pos);
    body->SetFrameRefToAbs(chrono::ChFramed(ref_pos, mount_rot * spec.rot));
    body->SetFixed(false);

    // One visual path for every arm body: a triangle mesh we load ourselves.
    // ChVisualShapeModelFile cannot be coloured in the sensor view -- ChOptixEngine
    // reloads the OBJ into a fresh shape and discards its materials -- so owning the
    // mesh is what lets both renderers read the same material.
    auto mesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(
        shapes_dir + spec.mesh, true, true);
    if (!mesh)
        throw std::runtime_error("Cannot load arm mesh: " + shapes_dir + spec.mesh);
    if (!spec.finger && geometry_scale != 1.0)
        mesh->Transform(chrono::VNULL, chrono::ChMatrix33<>(geometry_scale));

    auto visual = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    visual->SetMesh(mesh);
    visual->SetMutable(false);
    // Colour is applied once at the end of the constructor; see the GetBodies() loop.
    body->AddVisualShape(visual, chrono::ChFramed(chrono::VNULL, chrono::QUNIT));

    if (spec.finger) {
        // Match the system's contact method: an NSC pad in an SMC system (or vice
        // versa) silently fails the finger-vs-rock contact. High friction so the
        // position-driven pads grip the rock instead of sliding.
        auto mat = MakeContactMaterial(system->GetContactMethod(), 0.9f);
        mat->SetRollingFriction(0.5f);
        body->AddCollisionShape(chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat, 0.005, 0.13, 0.01),
                                chrono::ChFramed(chrono::ChVector3d(-0.106, 0.08, 0), chrono::QUNIT));
    }
    body->EnableCollision(spec.finger);

    system->AddBody(body);
    return body;
}

template <class LinkT>
std::shared_ptr<LinkT> AddLink(chrono::ChSystem* system,
                               std::shared_ptr<chrono::ChBody> body_1,
                               std::shared_ptr<chrono::ChBody> body_2,
                               const chrono::ChFramed& frame) {
    auto link = chrono_types::make_shared<LinkT>();
    link->Initialize(body_1, body_2, frame);
    system->AddLink(link);
    return link;
}

std::shared_ptr<chrono::ChBodyAuxRef> RequireBody(chrono::ChSystem* system, const std::string& name) {
    auto body = std::dynamic_pointer_cast<chrono::ChBodyAuxRef>(system->SearchBody(name));
    if (!body)
        throw std::runtime_error("Imported LRV arm is missing body: " + name);
    return body;
}

}  // namespace

LrvArm::LrvArm(chrono::ChSystem* system,
               std::shared_ptr<chrono::ChBody> chassis_body,
               const std::string& amd_uw_data_path,
               const chrono::ChVector3d& mount_pos,
               const chrono::ChQuaternion<>& mount_rot,
               bool import_solidworks,
               const std::string& name_prefix,
               const std::string& arm_model_relative_path,
               const std::string& shapes_relative_path,
               bool parked_rigid,
               double geometry_scale)
    : m_system(system),
      m_chassis_body(std::move(chassis_body)),
      m_geometry_scale(geometry_scale),
      // Every rank writes to one stdout, so an untagged "[LrvArm]" line cannot be
      // attributed to a machine. Four builders interleaving grab diagnostics in one
      // file is how a 0.76 m miss on one of them gets read as a property of another.
      m_log_tag(name_prefix.empty() ? std::string("rover") : name_prefix) {
    if (m_geometry_scale <= 0.0)
        throw std::invalid_argument("Arm geometry scale must be positive.");
    if (m_chassis_body)
        m_mount_rot_chassis = m_chassis_body->GetRot().GetConjugate() * mount_rot;

    std::string data_path = amd_uw_data_path;
    if (!data_path.empty() && data_path.back() != '/')
        data_path += "/";
    const std::string shapes_dir = data_path + shapes_relative_path;
    bool imported = false;
#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
    std::string arm_file;
    if (import_solidworks && m_geometry_scale == 1.0) {
        arm_file = data_path + arm_model_relative_path;

        {
            chrono::parsers::ChPythonEngine importer;
#ifdef PYCHRONO_MODULE_DIR
            // The exported SolidWorks file does `import pychrono`. The embedded
            // interpreter only finds it if the Chrono build's python bindings are on
            // sys.path, so add the build-configured location (idempotent if the
            // environment already exports it via PYTHONPATH).
            importer.Run(std::string("import sys\n"
                                     "p = r'") + PYCHRONO_MODULE_DIR + "'\n"
                         "if p not in sys.path:\n"
                         "    sys.path.insert(0, p)\n");
#endif
            importer.ImportSolidWorksSystem(arm_file, *system);
        }

        m_end_effector = RequireBody(system, "endeffector-1");
        m_biceps = RequireBody(system, "bicep-1");
        m_base = RequireBody(system, "base-1");
        m_shoulder = RequireBody(system, "shoulder-1");
        m_elbow = RequireBody(system, "elbow-1");
        m_wrist = RequireBody(system, "wrist-1");
        m_finger_2 = RequireBody(system, "finger-2");
        m_finger_1 = RequireBody(system, "finger-1");

        // Place imported bodies at exactly the manual-construction poses before
        // building joints from the shared mount-based frames below.
        {
            const std::array<std::shared_ptr<chrono::ChBodyAuxRef>, 8> imported_bodies = {
                m_end_effector, m_biceps, m_base, m_shoulder, m_elbow, m_wrist, m_finger_2, m_finger_1};
            for (size_t i = 0; i < imported_bodies.size(); ++i) {
                const auto& spec = arm_bodies[i];
                const chrono::ChVector3d P =
                    (std::string(spec.name) == "base-1") ? mount_pos : mount_pos + mount_rot.Rotate(spec.pos);
                const chrono::ChQuaternion<> R = mount_rot * spec.rot;
                imported_bodies[i]->SetFixed(false);
                imported_bodies[i]->SetFrameRefToAbs(chrono::ChFramed(P, R));
            }
        }
        imported = true;
    }
#endif
    if (!imported) {
        m_end_effector =
            CreateBody(system, arm_bodies[0], shapes_dir, mount_pos, mount_rot, name_prefix, m_geometry_scale);
        m_biceps =
            CreateBody(system, arm_bodies[1], shapes_dir, mount_pos, mount_rot, name_prefix, m_geometry_scale);
        m_base =
            CreateBody(system, arm_bodies[2], shapes_dir, mount_pos, mount_rot, name_prefix, m_geometry_scale);
        m_shoulder =
            CreateBody(system, arm_bodies[3], shapes_dir, mount_pos, mount_rot, name_prefix, m_geometry_scale);
        m_elbow =
            CreateBody(system, arm_bodies[4], shapes_dir, mount_pos, mount_rot, name_prefix, m_geometry_scale);
        m_wrist =
            CreateBody(system, arm_bodies[5], shapes_dir, mount_pos, mount_rot, name_prefix, m_geometry_scale);
        m_finger_2 =
            CreateBody(system, arm_bodies[6], shapes_dir, mount_pos, mount_rot, name_prefix, m_geometry_scale);
        m_finger_1 =
            CreateBody(system, arm_bodies[7], shapes_dir, mount_pos, mount_rot, name_prefix, m_geometry_scale);
    }

    // One grey for every arm, applied here because this is the only point both
    // construction paths reach: the SolidWorks importer builds its own visual shapes,
    // and the LRV arm takes that path. See ArmGrey.
    for (const auto& arm_body : GetBodies())
        ApplyColorToVisualShapes(arm_body, ArmGrey());

    if (parked_rigid) {
        const std::array<std::shared_ptr<chrono::ChBodyAuxRef>, 8> parked_bodies = {
            m_end_effector, m_biceps, m_base, m_shoulder, m_elbow, m_wrist, m_finger_2, m_finger_1};
        for (const auto& body : parked_bodies) {
            body->EnableCollision(false);
            body->SetFixed(true);
        }
        return;
    }

    // Both builds now place the arm bodies at the same mount-based poses before
    // this point, so the joints use identical mount-based frames.
    const auto joint_base_shoulder =
        TransformFrame({{-5.74189588383473e-19, 7.96390791413929e-18, 0.127}, chrono::QUNIT},
                       mount_pos, mount_rot, m_geometry_scale);
    const auto joint_shoulder_biceps =
        TransformFrame({{-8.07010409222406e-17, 1.5234866438078e-16, 0.325155513123522},
                        {1.17756934401283e-16, -1.17756934401283e-16, 0.707106781186548, -0.707106781186547}},
                       mount_pos, mount_rot, m_geometry_scale);
    const auto joint_biceps_elbow =
        TransformFrame({{-1.27, -2.66188598930217e-16, 0.325155513123522},
                        {0.707106781186548, -0.707106781186547, -1.17756934401283e-16, 1.17756934401283e-16}},
                       mount_pos, mount_rot, m_geometry_scale);
    const auto joint_elbow_effector =
        TransformFrame({{-2.413, -0.0190500000000001, 0.325155513123522},
                        {0.707106781186548, -0.707106781186547, -2.17894099802631e-33, 2.17894099802631e-33}},
                       mount_pos, mount_rot, m_geometry_scale);
    const auto joint_effector =
        TransformFrame({{-2.6924, -9.00811551237124e-16, 0.325155513123523},
                        {-3.92523114670944e-17, 3.92523114670943e-17, -0.707106781186547, 0.707106781186548}},
                       mount_pos, mount_rot, m_geometry_scale);

    m_wrist_lock = AddLink<chrono::ChLinkLockLock>(system, m_end_effector, m_wrist, joint_effector);
    m_motor_base_shoulder =
        AddLink<chrono::ChLinkMotorRotationAngle>(system, m_base, m_shoulder, joint_base_shoulder);
    m_motor_shoulder_biceps =
        AddLink<chrono::ChLinkMotorRotationAngle>(system, m_shoulder, m_biceps, joint_shoulder_biceps);
    m_motor_biceps_elbow = AddLink<chrono::ChLinkMotorRotationAngle>(system, m_biceps, m_elbow, joint_biceps_elbow);
    m_motor_elbow_effector =
        AddLink<chrono::ChLinkMotorRotationAngle>(system, m_elbow, m_end_effector, joint_elbow_effector);
    m_motor_finger_1 =
        AddLink<chrono::ChLinkMotorLinearPosition>(system, m_end_effector, m_finger_1, joint_effector);
    m_motor_finger_2 =
        AddLink<chrono::ChLinkMotorLinearPosition>(system, m_end_effector, m_finger_2, joint_effector);

    const std::array<std::shared_ptr<chrono::ChLinkMotorLinearPosition>, 2> finger_motors = {
        m_motor_finger_1, m_motor_finger_2};
    const std::array<double, 2> finger_sign = {-1.0, 1.0};
    for (int i = 0; i < 2; ++i) {
        m_finger_fn[i] = chrono_types::make_shared<chrono::ChFunctionSetpoint>();
        m_finger_fn[i]->SetSetpointAndDerivatives(finger_sign[i] * m_applied_close_pos, 0.0, 0.0);
        finger_motors[i]->SetMotionFunction(m_finger_fn[i]);
    }

    // One setpoint function per joint, attached once and then mutated in place by
    // AdvanceJointCommands. Attaching a freshly allocated function every step would
    // also work but allocates four shared_ptrs per arm per 5e-4 s step.
    //
    // The motors are built at angle 0, which in theta[] terms is {-pi, 0, 0, 0} --
    // the same value m_cmd_theta / m_applied_theta are initialised to, so the arm
    // starts already at its commanded pose and the first step demands nothing.
    const std::array<std::shared_ptr<chrono::ChLinkMotorRotationAngle>, 4> joint_motors = {
        m_motor_base_shoulder, m_motor_shoulder_biceps, m_motor_biceps_elbow, m_motor_elbow_effector};
    for (int i = 0; i < 4; ++i) {
        m_joint_fn[i] = chrono_types::make_shared<chrono::ChFunctionSetpoint>();
        m_joint_fn[i]->SetSetpointAndDerivatives(MotorAngleForJoint(i, m_applied_theta[i]), 0.0, 0.0);
        joint_motors[i]->SetAngleFunction(m_joint_fn[i]);
    }

    m_chassis_lock =
        AddLink<chrono::ChLinkLockLock>(system, m_chassis_body, m_base, chrono::ChFramed(mount_pos, mount_rot));

#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
    if (imported)
        std::cout << "[LrvArm " + m_log_tag + "] imported SolidWorks arm via ChPythonEngine: " << arm_file << "\n";
#endif
}

bool LrvArm::StartPickPlace(double command_seq,
                            int target_index,
                            std::shared_ptr<chrono::ChBodyAuxRef> rock,
                            const chrono::ChVector3d& grab_target_world,
                            const chrono::ChVector3d& place_target_world,
                            double time,
                            const std::array<double, 4>* grab_theta_override,
                            const std::array<double, 4>* place_theta_override) {
    if (!rock) {
        m_target_rock.reset();
        m_status.command_seq = command_seq;
        m_status.target_index = target_index;
        FinishFailed(1);
        return false;
    }

    RemoveRockLock();
    OpenGripper();

    m_status.command_seq = command_seq;
    m_status.target_index = target_index;
    m_status.state = 1;
    m_status.success = false;
    m_status.error_code = 0;
    m_target_rock = std::move(rock);
    // Freeze the rock in place while the arm servos onto it so the multi-step
    // approach cannot shove it, but keep collision on like the Python path so
    // the fingers cannot ghost through before the contact-triggered lock.
    m_target_rock->SetFixed(true);
    m_target_rock->EnableCollision(true);
    m_grab_target_world = grab_target_world;
    m_place_target_world = place_target_world;
    m_start_time = time;
    m_phase_time = time;
    m_next_tick = time;
    m_close_pos = 0.0;
    m_bent_arm = false;
    m_contact_seen = false;

    // Python is the sole IK authority: both the grab and place poses must arrive
    // in the arm_cmd. The C++ IK and closed-loop reach correction have been
    // removed, so a command that is missing a solved pose fails cleanly instead
    // of silently self-solving with a different solver/frame.
    if (!grab_theta_override) {
        std::cout << "[LrvArm " + m_log_tag + "] no grab theta in command -> failing (C++ IK removed)\n";
        FinishFailed(2);
        return false;
    }
    if (!place_theta_override) {
        std::cout << "[LrvArm " + m_log_tag + "] no place theta in command -> failing (C++ IK removed)\n";
        FinishFailed(2);
        return false;
    }
    // Reject a pose the arm cannot legally hold BEFORE commanding the motors to it.
    // The existing divergence_abort check in APPROACH is too late: it measures the
    // gripper error only after the arm has already swung to the pose, which for a
    // target inside the rover means after the fingers have been driven into the
    // chassis. See min_grab_reach_xy.
    {
        const double min_reach = min_grab_reach_xy * m_geometry_scale;
        const double max_reach = max_grab_reach_xy * m_geometry_scale;
        const double min_local_z = min_grab_local_z * m_geometry_scale;
        const chrono::ChVector3d local = GetIkFrameRot().RotateBack(grab_target_world - GetIkFramePos());
        const double reach_xy = std::hypot(local.x(), local.y());
        if (reach_xy < min_reach || reach_xy > max_reach || local.z() < min_local_z) {
            std::cout << "[LrvArm " + m_log_tag + "] grab target OUT OF ENVELOPE: local=(" << local.x() << ", " << local.y()
                      << ", " << local.z() << ") reach_xy=" << reach_xy << " m, allowed [" << min_reach
                      << ", " << max_reach << "] and z >= " << min_local_z << " (geometry_scale "
                      << m_geometry_scale
                      << "). The vehicle is parked wrong for this rock; refusing to fold the arm through "
                         "itself to reach it.\n";
            FinishFailed(5);
            return false;
        }
    }

    m_grab_theta = *grab_theta_override;
    m_place_theta = *place_theta_override;

    std::cout << "[LrvArm " + m_log_tag + "] grab_theta=(" << m_grab_theta[0] << "," << m_grab_theta[1] << ","
              << m_grab_theta[2] << "," << m_grab_theta[3] << ") -> starting APPROACH\n";

    CommandJointAngles({m_grab_theta[0], m_grab_theta[1], 0.0, 0.0});
    m_phase = Phase::APPROACH;
    return true;
}

void LrvArm::Update(double time) {
    // Before the phase early-out: the slew has to keep running in IDLE/DONE/FAILED
    // too. STOWING calls FinishDone() after stow_hold_time, and the arm may still be
    // travelling to the stow pose at that moment -- stopping the slew there would
    // freeze it mid-swing and leave the motors holding a pose they never reached.
    AdvanceJointCommands(time);

    if (m_phase == Phase::IDLE || m_phase == Phase::DONE || m_phase == Phase::FAILED)
        return;

    if (time - m_start_time > total_timeout) {
        FinishFailed(4);
        return;
    }

    if (m_phase == Phase::APPROACH) {
        const double elapsed = time - m_phase_time;
        if (!m_bent_arm && elapsed > approach_2_time) {
            CommandJointAngles(m_grab_theta);
            m_bent_arm = true;
            m_phase_time = time;  // begin settle window for the first correction
            return;
        }

        if (!m_bent_arm)
            return;
        // Only judge the pose once the arm has actually ARRIVED and settled --
        // gate on gripper speed, not a short fixed timer. Measuring mid-swing was
        // what caused the spurious corrections (flinging the aim low/around before
        // homing back onto the rock). A time cap is the fallback if it never quiets.
        const double gripper_speed = (0.5 * (m_finger_1->GetPosDt() + m_finger_2->GetPosDt())).Length();
        const bool settled = gripper_speed < settle_speed_tol;
        if (elapsed < approach_min_settle || (!settled && elapsed < close_timeout))
            return;

        // Measure the actual settled gripper error against the true rock target.
        const auto gc = GripperCenter();
        const double err = (m_grab_target_world - gc).Length();

        // A grab pose Python solved should settle near the rock. A gross miss means
        // the pose was bad (unreachable / wrong frame); fail cleanly instead of
        // clamping the fingers onto empty space.
        if (err > divergence_abort * m_geometry_scale) {
            std::cout << "[LrvArm " + m_log_tag + "] grab pose diverged err=" << err << " (limit "
                      << divergence_abort * m_geometry_scale << ") -> failing\n";
            FinishFailed(2);
            return;
        }

        // ARRIVAL is a different question from DIVERGENCE, and it needs its own number.
        //
        // This used to be the only test, so anything inside divergence_abort proceeded to
        // close -- 2.0 m for the builder, since that constant scales with the arm. But the
        // jaws can only take a rock their pads actually touch: lock_finger_dist is 0.27 m
        // and does NOT scale, because the fingers stay 1x on the 2x builder arm. So the
        // gate that decided "I have arrived" was nearly eight times looser than the gate
        // that decides "I can grip", and the gap between them is a grab that closes on air.
        //
        // Measured, three times, on two builders and across runs -- the miss is remarkably
        // repeatable because it is a threshold and not a drift:
        //   [builder_2_] miss_xy=0.726 miss_z=-0.082  pad_force=0
        //   [builder_3_] miss_xy=0.735 miss_z=+0.089  pad_force=0
        //   APPROACH->CLOSING |gripper-target|=0.760615
        // Each time the aim point was exactly the rock's x/y, so nothing upstream was
        // wrong: the arm simply had not got there, and was told that was close enough.
        //
        // Settled-but-short now keeps waiting -- the joint motors are constraint motors
        // and do converge -- and only a timeout fails it, with the error named.
        // Unscaled: a finger-scale quantity, and set BETWEEN the two measured populations
        // rather than by taste. Grabs that go on to lock arrive at 0.016-0.116 m (one
        // locked from 0.1159, and 0.1385 was rejected by a first cut at 0.12 that was
        // plainly too tight); grabs that close on air arrive at 0.726-0.760 m. There is
        // half a metre of clear air between those, so anything in 0.2-0.5 separates them.
        // 0.20 takes the low end, staying well under the 0.27 m the lock itself needs.
        constexpr double grab_accept_dist = 0.20;
        if (err > grab_accept_dist) {
            if (elapsed < close_timeout)
                return;  // still converging; give it the rest of its window
            std::cout << "[LrvArm " + m_log_tag + "] grab pose never arrived: err=" << err
                      << " m after " << elapsed << " s (accept " << grab_accept_dist
                      << " m, lock needs " << lock_finger_dist << " m) -> failing\n";
            FinishFailed(2);
            return;
        }

        const auto rref = m_target_rock ? m_target_rock->GetFrameRefToAbs().GetPos() : chrono::VNULL;
        std::cout << "[LrvArm " + m_log_tag + "] APPROACH->CLOSING t=" << time
                  << " |gripper-target|=" << err
                  << " |gripper-rockREF|xy=" << std::hypot(gc.x() - rref.x(), gc.y() - rref.y()) << "\n";
        m_phase = Phase::CLOSING;
        m_phase_time = time;
        m_next_tick = time;
        return;
    }

    if (time < m_next_tick)
        return;
    m_next_tick = time + control_dt;

    if (m_phase == Phase::CLOSING) {
        m_close_pos = std::min(m_close_pos + finger_close_speed * control_dt, finger_close_pos);
        CommandFingerPosition(m_close_pos);

        const double actual_sep = (m_finger_1->GetPos() - m_finger_2->GetPos()).Length();
        const double pad_force =
            std::max(m_finger_1->GetContactForce().Length(), m_finger_2->GetContactForce().Length());
        const bool target_lockable =
            m_target_rock &&
            (m_target_rock->GetPos() - m_finger_1->GetPos()).Length() < lock_finger_dist &&
            (m_target_rock->GetPos() - m_finger_2->GetPos()).Length() < lock_finger_dist;
        if (target_lockable && pad_force > grip_force_tol)
            m_contact_seen = true;

        if (m_contact_seen && target_lockable && actual_sep <= finger_grasp_sep) {
            m_close_pos = std::min(finger_close_pos, 0.5 * (finger_open_sep - actual_sep) + 0.002);
            CommandFingerPosition(m_close_pos);
            if (TryLockRock()) {
                std::cout << "[LrvArm " + m_log_tag + "] LOCKED t=" << time << " actual_sep=" << actual_sep
                          << " pad_force=" << pad_force << "\n";
                // Unfreezing and disabling collision now happen inside TryLockRock,
                // BEFORE the weld is built rather than one line after it -- see the
                // comment there.
                m_lift_angle = m_grab_theta[1];
                m_phase = Phase::LIFTING;
                m_phase_time = time;
                return;
            }
        }

        if (m_close_pos >= finger_close_pos) {
            const double d1 = m_target_rock ? (m_target_rock->GetPos() - m_finger_1->GetPos()).Length() : -1.0;
            const double d2 = m_target_rock ? (m_target_rock->GetPos() - m_finger_2->GetPos()).Length() : -1.0;
            // Split the miss into its horizontal and vertical parts, and report where the
            // rock and the aim point actually are. A single 3D distance cannot tell a
            // gripper that went to the wrong PLACE from one that went to the right place
            // at the wrong HEIGHT, and those have nothing to do with each other: the first
            // is a targeting fault, the second is the grab_z_offset calibration. Reading
            // one as the other has cost this project two wrong diagnoses.
            double dxy = -1.0, dz = 0.0;
            chrono::ChVector3d rock_pos, mid;
            if (m_target_rock) {
                rock_pos = m_target_rock->GetPos();
                mid = 0.5 * (m_finger_1->GetPos() + m_finger_2->GetPos());
                dxy = std::hypot(rock_pos.x() - mid.x(), rock_pos.y() - mid.y());
                dz = mid.z() - rock_pos.z();
            }
            std::cout << "[LrvArm " + m_log_tag + "] GRAB FAILED(3) t=" << time << " actual_sep=" << actual_sep
                      << " (grasp_sep=" << finger_grasp_sep << ")"
                      << " miss_xy=" << dxy << " miss_z=" << dz
                      << " rock=(" << rock_pos.x() << "," << rock_pos.y() << "," << rock_pos.z() << ")"
                      << " aim=(" << m_grab_target_world.x() << "," << m_grab_target_world.y() << "," << m_grab_target_world.z()
                      << ")"
                      << " dist_finger1_rock=" << d1 << " dist_finger2_rock=" << d2
                      << " pad_force=" << pad_force
                      << " (lock_dist=" << lock_finger_dist << ")\n";
            FinishFailed(3);
        }
        return;
    }

    if (m_phase == Phase::LIFTING) {
        if (time - m_phase_time < lift_delay)
            return;

        const double diff = lift_theta2 - m_lift_angle;
        if (std::abs(diff) <= lift_speed * control_dt) {
            m_lift_angle = lift_theta2;
            CommandJointAngles({m_grab_theta[0], m_lift_angle, m_grab_theta[2], m_grab_theta[3]});
            CommandJointAngles(m_place_theta);
            m_phase = Phase::PLACING;
            m_phase_time = time;
            return;
        }

        m_lift_angle += std::copysign(lift_speed * control_dt, diff);
        CommandJointAngles({m_grab_theta[0], m_lift_angle, m_grab_theta[2], m_grab_theta[3]});
        return;
    }

    if (m_phase == Phase::PLACING) {
        const double elapsed = time - m_phase_time;
        const double place_err = (GripperCenter() - m_place_target_world).Length();
        const double gripper_speed = (0.5 * (m_finger_1->GetPosDt() + m_finger_2->GetPosDt())).Length();
        const bool settled = gripper_speed < settle_speed_tol;
        // Release ONLY once the arm has actually come to rest at the place pose.
        // The rigid single-shot motors snap fast through the place pose (~1.4 m/s),
        // so releasing on proximity (place_err < place_tol) fires mid-swing and
        // FLINGS the rock off the bed (seen as |rock-place|~1.8 with the rock
        // launched a meter away). Waiting for `settled` drops it straight down.
        if ((elapsed >= place_min_settle && settled) || elapsed > place_timeout) {
            std::cout << "[LrvArm " + m_log_tag + "] PLACING->RELEASING t=" << time << " elapsed=" << elapsed
                      << " |gripper-place|=" << place_err << " gripper_speed=" << gripper_speed
                      << (elapsed > place_timeout ? " (timeout)" : "") << "\n";
            OpenGripper();
            m_phase = Phase::RELEASING;
            m_phase_time = time;
        }
        return;
    }

    if (m_phase == Phase::RELEASING) {
        if (time - m_phase_time > release_hold_time) {
            CommandJointAngles(stow_theta);
            m_phase = Phase::STOWING;
            m_phase_time = time;
        }
        return;
    }

    if (m_phase == Phase::STOWING && time - m_phase_time > stow_hold_time) {
        FinishDone();
    }
}

bool LrvArm::IsBusy() const {
    return m_phase == Phase::APPROACH || m_phase == Phase::CLOSING || m_phase == Phase::LIFTING ||
           m_phase == Phase::PLACING || m_phase == Phase::RELEASING || m_phase == Phase::STOWING;
}

ArmStatusSnapshot LrvArm::GetStatus() const {
    return m_status;
}

const char* LrvArm::GetPhaseName() const {
    switch (m_phase) {
        case Phase::IDLE: return "IDLE";
        case Phase::APPROACH: return "APPROACH";
        case Phase::CLOSING: return "CLOSING";
        case Phase::LIFTING: return "LIFTING";
        case Phase::PLACING: return "PLACING";
        case Phase::RELEASING: return "RELEASING";
        case Phase::STOWING: return "STOWING";
        case Phase::DONE: return "DONE";
        case Phase::FAILED: return "FAILED";
    }
    return "?";
}

chrono::ChVector3d LrvArm::GetIkFramePos() const {
    return m_base ? m_base->GetPos() : chrono::VNULL;
}

double LrvArm::BaseOffsetFromChassis() const {
    if (!m_base || !m_chassis_body)
        return 0.0;
    return (m_base->GetPos() - m_chassis_body->GetPos()).Length();
}

chrono::ChQuaternion<> LrvArm::GetIkFrameRot() const {
    return m_chassis_body ? m_chassis_body->GetRot() * m_mount_rot_chassis : m_mount_rot_chassis;
}

std::shared_ptr<chrono::ChBodyAuxRef> LrvArm::GetActiveRock() const {
    // Only while the arm is actively positioning onto / holding the rock. Once
    // it is being released (RELEASING/STOWING) the rig may manage it again.
    if (m_phase == Phase::APPROACH || m_phase == Phase::CLOSING || m_phase == Phase::LIFTING ||
        m_phase == Phase::PLACING) {
        return m_target_rock;
    }
    return nullptr;
}

void LrvArm::ForgetTargetRock() {
    // Refuse while the rock is still welded on: dropping the reference then would leave
    // the weld pointing at a body nothing else tracks, and RemoveRockLock could never
    // restore its collision.
    if (m_rock_lock)
        return;
    m_target_rock.reset();
}

void LrvArm::SetJointTargets(const std::array<double, 4>& theta) {
    CommandJointAngles(theta);
}

void LrvArm::SetFingerClosure(double close_pos) {
    m_close_pos = std::clamp(close_pos, 0.0, finger_close_pos);
    CommandFingerPosition(m_close_pos);
}

ArmActuatorSnapshot LrvArm::GetActuatorSnapshot() const {
    ArmActuatorSnapshot snapshot;
    snapshot.joint_angles = {
        -m_motor_base_shoulder->GetMotorAngle() - chrono::CH_PI,
        m_motor_shoulder_biceps->GetMotorAngle(),
        -m_motor_biceps_elbow->GetMotorAngle(),
        -m_motor_elbow_effector->GetMotorAngle()};
    snapshot.finger_positions = {
        m_motor_finger_1->GetMotorPos(),
        m_motor_finger_2->GetMotorPos()};
    snapshot.end_effector_position = m_end_effector->GetPos();
    return snapshot;
}

std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> LrvArm::GetBodies() const {
    return {
        m_end_effector,
        m_biceps,
        m_base,
        m_shoulder,
        m_elbow,
        m_wrist,
        m_finger_2,
        m_finger_1,
    };
}

// Sign/offset convention between the theta[] used everywhere in this file and the
// raw motor angles. GetActuatorSnapshot inverts exactly this.
double LrvArm::MotorAngleForJoint(int joint, double theta) {
    switch (joint) {
        case 0: return -theta - chrono::CH_PI;
        case 1: return theta;
        default: return -theta;
    }
}

double LrvArm::MotorRateForJoint(int joint, double theta_rate) {
    return (joint == 1) ? theta_rate : -theta_rate;
}

// Record the TARGET pose only. Never write the motors here.
//
// These are ChLinkMotorRotationAngle: the function value IS the commanded joint
// position, so swapping in a new constant is a position discontinuity -- the same
// hazard as the rack-pinion steering and the trailer bed, both of which were given
// bounded slew rates for exactly this reason. Here the steps are large: rest or
// stow (-pi) to a grab pose near -1.2 rad is ~1.9 rad demanded in a single 5e-4 s
// step, i.e. a commanded ~3900 rad/s, while ChFunctionConst reports a derivative of
// zero so the velocity-level constraint simultaneously says "hold still". The
// solver absorbs that contradiction as an impulse, it reacts through the base into
// the ChLinkLockLock welding the arm to the chassis, and that is the arm-base
// divergence to NaN. The PLACING phase already documented the visible half of this
// ("the rigid single-shot motors snap fast through the place pose (~1.4 m/s)").
void LrvArm::CommandJointAngles(const std::array<double, 4>& theta) {
    m_cmd_theta = theta;

    // Base yaw takes the SHORT way round.
    //
    // theta[0] arrives from the Python IK, which is exact (fk_err = 0) but returns
    // the angle in (-pi, pi]. Mapped through -theta - pi, a solution of -1.2 rad
    // becomes a motor angle of -1.94 rad while +1.2 rad becomes -4.34 rad -- the
    // same physical pose 2*pi away, but 249 degrees of travel instead of 111, and
    // in the same rotational direction, so the arm sweeps the long way round
    // through its own chassis and trailer instead of straight out to the side.
    //
    // Which one the IK returns depends only on which side of the rover the rock
    // ended up (rock_side_offset_m puts local_target.y at -2 or +2), so a rank
    // approaching from the mirrored side had every grab fail: the arm either
    // diverged en route (err 1.9 m) or arrived late and closed on empty air 0.4 m
    // short. Ranks that happened to approach from the other side worked perfectly
    // with an identical solver, which is what made it look like a per-rank fault.
    //
    // Only joint 0 is wrapped. It is a full revolute, so every 2*pi-equivalent is
    // the same pose; the shoulder/elbow/wrist are not, and wrapping those would
    // silently select a folded-back configuration rather than a shorter path.
    const double delta = m_cmd_theta[0] - m_applied_theta[0];
    m_cmd_theta[0] = m_applied_theta[0] + std::remainder(delta, chrono::CH_2PI);
}

void LrvArm::AdvanceJointCommands(double time) {
    const double dt = (m_last_cmd_time >= 0.0) ? (time - m_last_cmd_time) : 0.0;
    m_last_cmd_time = time;
    if (dt <= 0.0)
        return;

    const double max_step = joint_slew_rate * dt;
    for (int i = 0; i < 4; ++i) {
        const double diff = m_cmd_theta[i] - m_applied_theta[i];
        const double step = std::clamp(diff, -max_step, max_step);
        m_applied_theta[i] += step;
        // Hand the motor a consistent position AND velocity. ChFunctionConst
        // always reports zero derivative, which is what makes even a small
        // position increment inconsistent at the velocity level.
        m_joint_fn[i]->SetSetpointAndDerivatives(MotorAngleForJoint(i, m_applied_theta[i]),
                                                 MotorRateForJoint(i, step / dt),
                                                 0.0);
    }

    const double finger_diff = m_cmd_close_pos - m_applied_close_pos;
    const double finger_step = std::clamp(finger_diff, -finger_slew_rate * dt, finger_slew_rate * dt);
    m_applied_close_pos += finger_step;
    const double finger_rate = finger_step / dt;
    m_finger_fn[0]->SetSetpointAndDerivatives(-m_applied_close_pos, -finger_rate, 0.0);
    m_finger_fn[1]->SetSetpointAndDerivatives(m_applied_close_pos, finger_rate, 0.0);
}

// Target only; the slew in AdvanceJointCommands writes the motors.
void LrvArm::CommandFingerPosition(double close_pos) {
    m_cmd_close_pos = close_pos;
}

void LrvArm::OpenGripper() {
    RemoveRockLock();
    m_close_pos = 0.0;
    CommandFingerPosition(0.0);
}

bool LrvArm::TryLockRock() {
    if (!m_target_rock || m_rock_lock)
        return static_cast<bool>(m_rock_lock);

    const auto rock_pos = m_target_rock->GetPos();
    if ((rock_pos - m_finger_1->GetPos()).Length() >= lock_finger_dist ||
        (rock_pos - m_finger_2->GetPos()).Length() >= lock_finger_dist) {
        return false;
    }

    // Give the rock its state variables back BEFORE welding it to the end effector.
    //
    // The rock is held SetFixed(true) through APPROACH so the gripper cannot shove
    // it. A fixed body has no variables in the solver, so initialising a
    // ChLinkLockLock against it builds the constraint against a body that is not
    // part of the system's DOFs -- and the caller then unfixed it on the very next
    // line, meaning the weld's first solved step was also the first step in which
    // its second body existed. Unfixing first makes the weld's Jacobian describe
    // the body it will actually constrain.
    //
    // Collision goes off at the same moment and for the same reason it always did:
    // once bonded, the rock must not fight the fingers or the terrain.
    m_target_rock->SetFixed(false);
    m_target_rock->EnableCollision(false);

    const auto midpoint = GripperCenter();
    m_rock_lock = chrono_types::make_shared<chrono::ChLinkLockLock>();
    m_rock_lock->Initialize(m_end_effector, m_target_rock, chrono::ChFramed(midpoint, chrono::QUNIT));
    m_system->AddLink(m_rock_lock);
    return true;
}

void LrvArm::RemoveRockLock() {
    if (m_rock_lock) {
        m_system->RemoveLink(m_rock_lock);
        m_rock_lock.reset();
    }
    if (m_target_rock) {
        m_target_rock->SetFixed(false);
        m_target_rock->EnableCollision(true);
    }
}

chrono::ChVector3d LrvArm::GripperCenter() const {
    return 0.5 * (m_finger_1->GetPos() + m_finger_2->GetPos());
}

void LrvArm::FinishDone() {
    if (m_target_rock) {
        const auto rp = m_target_rock->GetPos();
        std::cout << "[LrvArm " + m_log_tag + "] DONE rock_final=(" << rp.x() << "," << rp.y() << "," << rp.z() << ")"
                  << " place_target=(" << m_place_target_world.x() << "," << m_place_target_world.y() << ","
                  << m_place_target_world.z() << ")"
                  << " |rock-place|=" << (rp - m_place_target_world).Length() << "\n";
    }
    m_phase = Phase::DONE;
    m_status.state = 2;
    m_status.success = true;
    m_status.error_code = 0;
}

void LrvArm::FinishFailed(int error_code) {
    OpenGripper();
    m_phase = Phase::FAILED;
    m_status.state = 3;
    m_status.success = false;
    m_status.error_code = error_code;
}

}  // namespace amd_uw
