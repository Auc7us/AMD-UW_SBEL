#include "LrvArm.h"

#include <array>
#include <memory>
#include <string>

#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChTypes.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/physics/ChContactMaterial.h"

namespace amd_uw {

namespace {

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

}  // namespace

LrvArm::LrvArm(chrono::ChSystem* system,
               std::shared_ptr<chrono::ChBody> chassis_body,
               const std::string& amd_uw_data_path,
               const chrono::ChVector3d& mount_pos,
               const chrono::ChQuaternion<>& mount_rot) {
    std::string data_path = amd_uw_data_path;
    if (!data_path.empty() && data_path.back() != '/')
        data_path += "/";
    const std::string shapes_dir = data_path + "lrv_robotarm/lrv_arm_shapes/";

    m_end_effector = CreateBody(system, arm_bodies[0], shapes_dir, mount_pos, mount_rot);
    m_biceps = CreateBody(system, arm_bodies[1], shapes_dir, mount_pos, mount_rot);
    m_base = CreateBody(system, arm_bodies[2], shapes_dir, mount_pos, mount_rot);
    m_shoulder = CreateBody(system, arm_bodies[3], shapes_dir, mount_pos, mount_rot);
    m_elbow = CreateBody(system, arm_bodies[4], shapes_dir, mount_pos, mount_rot);
    m_wrist = CreateBody(system, arm_bodies[5], shapes_dir, mount_pos, mount_rot);
    m_finger_2 = CreateBody(system, arm_bodies[6], shapes_dir, mount_pos, mount_rot);
    m_finger_1 = CreateBody(system, arm_bodies[7], shapes_dir, mount_pos, mount_rot);

    const auto joint_base_shoulder =
        TransformFrame({{-5.74189588383473e-19, 7.96390791413929e-18, 0.127}, chrono::QUNIT}, mount_pos, mount_rot);
    const auto joint_shoulder_biceps =
        TransformFrame({{-8.07010409222406e-17, 1.5234866438078e-16, 0.325155513123522},
                        {1.17756934401283e-16, -1.17756934401283e-16, 0.707106781186548, -0.707106781186547}},
                       mount_pos, mount_rot);
    const auto joint_biceps_elbow =
        TransformFrame({{-1.27, -2.66188598930217e-16, 0.325155513123522},
                        {0.707106781186548, -0.707106781186547, -1.17756934401283e-16, 1.17756934401283e-16}},
                       mount_pos, mount_rot);
    const auto joint_elbow_effector =
        TransformFrame({{-2.413, -0.0190500000000001, 0.325155513123522},
                        {0.707106781186548, -0.707106781186547, -2.17894099802631e-33, 2.17894099802631e-33}},
                       mount_pos, mount_rot);
    const auto joint_effector =
        TransformFrame({{-2.6924, -9.00811551237124e-16, 0.325155513123523},
                        {-3.92523114670944e-17, 3.92523114670943e-17, -0.707106781186547, 0.707106781186548}},
                       mount_pos, mount_rot);

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

    m_chassis_lock = AddLink<chrono::ChLinkLockLock>(system, chassis_body, m_base, chrono::ChFramed(mount_pos, mount_rot));
}

}  // namespace amd_uw
