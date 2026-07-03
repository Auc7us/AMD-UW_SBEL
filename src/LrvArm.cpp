#include "LrvArm.h"

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChTypes.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/physics/ChContactMaterial.h"
#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
#include "chrono_parsers/ChParserPython.h"
#endif

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
                                const chrono::ChQuaternion<>& mount_rot) {
    return chrono::ChFramed(mount_pos + mount_rot.Rotate(frame.pos), mount_rot * frame.rot);
}

std::shared_ptr<chrono::ChBodyAuxRef> CreateBody(chrono::ChSystem* system,
                                                 const BodySpec& spec,
                                                 const std::string& shapes_dir,
                                                 const chrono::ChVector3d& mount_pos,
                                                 const chrono::ChQuaternion<>& mount_rot) {
    auto body = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    body->SetName(spec.name);
    body->SetPos(spec.name == std::string("base-1") ? mount_pos : mount_pos + mount_rot.Rotate(spec.pos));
    body->SetRot(mount_rot * spec.rot);
    body->SetMass(spec.mass);
    body->SetInertiaXX(spec.inertia_xx);
    body->SetInertiaXY(spec.inertia_xy);
    body->SetFrameCOMToRef(chrono::ChFramed(spec.com, chrono::QUNIT));
    body->SetFixed(false);

    auto visual = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
    visual->SetFilename(shapes_dir + spec.mesh);
    body->AddVisualShape(visual, chrono::ChFramed(chrono::VNULL, chrono::QUNIT));

    if (spec.finger) {
        auto mat = chrono::ChContactMaterial::DefaultMaterial(chrono::ChContactMethod::NSC);
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

#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
void MoveImportedAssembly(const std::array<std::shared_ptr<chrono::ChBodyAuxRef>, 8>& bodies,
                          const chrono::ChVector3d& mount_pos,
                          const chrono::ChQuaternion<>& mount_rot) {
    for (const auto& body : bodies) {
        body->SetFixed(false);
        body->SetPos(mount_pos + body->GetPos());
    }

    // Match model/arm_model.py: after shifting the imported assembly, pin the
    // base body itself exactly at the requested mount point.
    bodies[2]->SetPos(mount_pos);

    for (const auto& body : bodies) {
        body->SetPos(mount_pos + mount_rot.Rotate(body->GetPos() - mount_pos));
        body->SetRot(mount_rot * body->GetRot());
    }
}
#endif

}  // namespace

LrvArm::LrvArm(chrono::ChSystem* system,
               std::shared_ptr<chrono::ChBody> chassis_body,
               const std::string& amd_uw_data_path,
               const chrono::ChVector3d& mount_pos,
               const chrono::ChQuaternion<>& mount_rot)
    : m_system(system), m_chassis_body(std::move(chassis_body)) {
    std::string data_path = amd_uw_data_path;
    if (!data_path.empty() && data_path.back() != '/')
        data_path += "/";
#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
    const std::string arm_file = data_path + "lrv_robotarm/lrv_arm.py";

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
#else
    const std::string shapes_dir = data_path + "lrv_robotarm/lrv_arm_shapes/";

    m_end_effector = CreateBody(system, arm_bodies[0], shapes_dir, mount_pos, mount_rot);
    m_biceps = CreateBody(system, arm_bodies[1], shapes_dir, mount_pos, mount_rot);
    m_base = CreateBody(system, arm_bodies[2], shapes_dir, mount_pos, mount_rot);
    m_shoulder = CreateBody(system, arm_bodies[3], shapes_dir, mount_pos, mount_rot);
    m_elbow = CreateBody(system, arm_bodies[4], shapes_dir, mount_pos, mount_rot);
    m_wrist = CreateBody(system, arm_bodies[5], shapes_dir, mount_pos, mount_rot);
    m_finger_2 = CreateBody(system, arm_bodies[6], shapes_dir, mount_pos, mount_rot);
    m_finger_1 = CreateBody(system, arm_bodies[7], shapes_dir, mount_pos, mount_rot);
#endif

    const auto joint_base_shoulder =
#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
        TransformFrame({{-5.74189588383473e-19, 7.96390791413929e-18, 0.127}, chrono::QUNIT}, chrono::VNULL, chrono::QUNIT);
#else
        TransformFrame({{-5.74189588383473e-19, 7.96390791413929e-18, 0.127}, chrono::QUNIT}, mount_pos, mount_rot);
#endif
    const auto joint_shoulder_biceps =
        TransformFrame({{-8.07010409222406e-17, 1.5234866438078e-16, 0.325155513123522},
                        {1.17756934401283e-16, -1.17756934401283e-16, 0.707106781186548, -0.707106781186547}},
#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
                       chrono::VNULL, chrono::QUNIT);
#else
                       mount_pos, mount_rot);
#endif
    const auto joint_biceps_elbow =
        TransformFrame({{-1.27, -2.66188598930217e-16, 0.325155513123522},
                        {0.707106781186548, -0.707106781186547, -1.17756934401283e-16, 1.17756934401283e-16}},
#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
                       chrono::VNULL, chrono::QUNIT);
#else
                       mount_pos, mount_rot);
#endif
    const auto joint_elbow_effector =
        TransformFrame({{-2.413, -0.0190500000000001, 0.325155513123522},
                        {0.707106781186548, -0.707106781186547, -2.17894099802631e-33, 2.17894099802631e-33}},
#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
                       chrono::VNULL, chrono::QUNIT);
#else
                       mount_pos, mount_rot);
#endif
    const auto joint_effector =
        TransformFrame({{-2.6924, -9.00811551237124e-16, 0.325155513123523},
                        {-3.92523114670944e-17, 3.92523114670943e-17, -0.707106781186547, 0.707106781186548}},
#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
                       chrono::VNULL, chrono::QUNIT);
#else
                       mount_pos, mount_rot);
#endif

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

    m_motor_finger_1->SetMotionFunction(chrono_types::make_shared<chrono::ChFunctionConst>(-0.15));
    m_motor_finger_2->SetMotionFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.15));

#ifdef AMD_UW_USE_SOLIDWORKS_IMPORTER
    MoveImportedAssembly({m_end_effector, m_biceps, m_base, m_shoulder, m_elbow, m_wrist, m_finger_2, m_finger_1},
                         mount_pos,
                         mount_rot);

    m_chassis_lock =
        AddLink<chrono::ChLinkLockLock>(system, m_chassis_body, m_base, chrono::ChFramed(m_base->GetPos(), m_base->GetRot()));

    std::cout << "[LrvArm] imported SolidWorks arm via ChPythonEngine: " << arm_file << "\n";
#else
    m_chassis_lock =
        AddLink<chrono::ChLinkLockLock>(system, m_chassis_body, m_base, chrono::ChFramed(mount_pos, mount_rot));
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
        std::cout << "[LrvArm] no grab theta in command -> failing (C++ IK removed)\n";
        FinishFailed(2);
        return false;
    }
    if (!place_theta_override) {
        std::cout << "[LrvArm] no place theta in command -> failing (C++ IK removed)\n";
        FinishFailed(2);
        return false;
    }
    m_grab_theta = *grab_theta_override;
    m_place_theta = *place_theta_override;

    std::cout << "[LrvArm] grab_theta=(" << m_grab_theta[0] << "," << m_grab_theta[1] << ","
              << m_grab_theta[2] << "," << m_grab_theta[3] << ") -> starting APPROACH\n";

    CommandJointAngles({m_grab_theta[0], m_grab_theta[1], 0.0, 0.0});
    m_phase = Phase::APPROACH;
    return true;
}

void LrvArm::Update(double time) {
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
        if (err > divergence_abort) {
            std::cout << "[LrvArm] grab pose diverged err=" << err << " -> failing\n";
            FinishFailed(2);
            return;
        }

        const auto rref = m_target_rock ? m_target_rock->GetFrameRefToAbs().GetPos() : chrono::VNULL;
        std::cout << "[LrvArm] APPROACH->CLOSING t=" << time
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
                std::cout << "[LrvArm] LOCKED t=" << time << " actual_sep=" << actual_sep
                          << " pad_force=" << pad_force << "\n";
                // Now bonded to the end-effector: unfreeze so it lifts with the
                // arm, but keep collision off so it doesn't fight terrain/fingers.
                m_target_rock->SetFixed(false);
                m_target_rock->EnableCollision(false);
                m_lift_angle = m_grab_theta[1];
                m_phase = Phase::LIFTING;
                m_phase_time = time;
                return;
            }
        }

        if (m_close_pos >= finger_close_pos) {
            const double d1 = m_target_rock ? (m_target_rock->GetPos() - m_finger_1->GetPos()).Length() : -1.0;
            const double d2 = m_target_rock ? (m_target_rock->GetPos() - m_finger_2->GetPos()).Length() : -1.0;
            std::cout << "[LrvArm] GRAB FAILED(3) t=" << time << " actual_sep=" << actual_sep
                      << " (grasp_sep=" << finger_grasp_sep << ")"
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
        if (place_err < place_tol || (elapsed >= place_min_settle && settled) || elapsed > place_timeout) {
            std::cout << "[LrvArm] PLACING->RELEASING t=" << time << " elapsed=" << elapsed
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

chrono::ChVector3d LrvArm::GetIkFramePos() const {
    return m_base ? m_base->GetPos() : chrono::VNULL;
}

chrono::ChQuaternion<> LrvArm::GetIkFrameRot() const {
    return m_chassis_body ? m_chassis_body->GetRot() : chrono::QUNIT;
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

void LrvArm::CommandJointAngles(const std::array<double, 4>& theta) {
    m_motor_base_shoulder->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(-theta[0] - chrono::CH_PI));
    m_motor_shoulder_biceps->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(theta[1]));
    m_motor_biceps_elbow->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(-theta[2]));
    m_motor_elbow_effector->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(-theta[3]));
}

void LrvArm::CommandFingerPosition(double close_pos) {
    m_motor_finger_1->SetMotionFunction(chrono_types::make_shared<chrono::ChFunctionConst>(-close_pos));
    m_motor_finger_2->SetMotionFunction(chrono_types::make_shared<chrono::ChFunctionConst>(close_pos));
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
        std::cout << "[LrvArm] DONE rock_final=(" << rp.x() << "," << rp.y() << "," << rp.z() << ")"
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
