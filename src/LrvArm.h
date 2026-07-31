#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystem.h"

namespace amd_uw {

struct ArmStatusSnapshot {
    double command_seq = 0.0;
    int state = 0;
    int target_index = -1;
    bool success = false;
    int error_code = 0;
};

struct ArmActuatorSnapshot {
    std::array<double, 4> joint_angles = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 2> finger_positions = {0.0, 0.0};
    chrono::ChVector3d end_effector_position;
};

class LrvArm {
  public:
    LrvArm(chrono::ChSystem* system,
           std::shared_ptr<chrono::ChBody> chassis_body,
           const std::string& amd_uw_data_path,
           const chrono::ChVector3d& mount_pos,
           const chrono::ChQuaternion<>& mount_rot,
           bool import_solidworks = true,
           const std::string& name_prefix = "",
           const std::string& arm_model_relative_path = "lrv_robotarm/lrv_arm.py",
           const std::string& shapes_relative_path = "lrv_robotarm/lrv_arm_shapes/",
           bool parked_rigid = false,
           double geometry_scale = 1.0);

    bool StartPickPlace(double command_seq,
                        int target_index,
                        std::shared_ptr<chrono::ChBodyAuxRef> rock,
                        const chrono::ChVector3d& grab_target_world,
                        const chrono::ChVector3d& place_target_world,
                        double time,
                        const std::array<double, 4>* grab_theta_override = nullptr,
                        const std::array<double, 4>* place_theta_override = nullptr);
    void Update(double time);
    bool IsBusy() const;
    ArmStatusSnapshot GetStatus() const;
    chrono::ChVector3d GetIkFramePos() const;
    chrono::ChQuaternion<> GetIkFrameRot() const;

    // Distance from the arm base body to the chassis it is mounted on. The mount is
    // rigid, so this is a constant; a change means the base body has drifted, and
    // the base body IS the IK frame origin published to the Python solver, so any
    // drift silently moves every IK target by the same amount.
    double BaseOffsetFromChassis() const;

    // Direct actuator interface used by the builder arm bridge and its
    // deterministic actuation test. Angles follow the same theta convention as
    // the Python IK output consumed by StartPickPlace.
    void SetJointTargets(const std::array<double, 4>& theta);
    void SetFingerClosure(double close_pos);
    ArmActuatorSnapshot GetActuatorSnapshot() const;
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> GetBodies() const;

    // The rock the arm is currently positioning onto / holding, if any. While
    // non-null the arm owns this rock's fixed/collision state (it is frozen so
    // the approach cannot knock it), so the rig must not re-toggle its collision.
    std::shared_ptr<chrono::ChBodyAuxRef> GetActiveRock() const;

  private:
    enum class Phase { IDLE, APPROACH, CLOSING, LIFTING, PLACING, RELEASING, STOWING, DONE, FAILED };

    void CommandJointAngles(const std::array<double, 4>& theta);
    void CommandFingerPosition(double close_pos);
    void OpenGripper();
    bool TryLockRock();
    void RemoveRockLock();
    chrono::ChVector3d GripperCenter() const;
    void FinishDone();
    void FinishFailed(int error_code);

    chrono::ChSystem* m_system;
    std::shared_ptr<chrono::ChBody> m_chassis_body;
    chrono::ChQuaternion<> m_mount_rot_chassis = chrono::QUNIT;
    double m_geometry_scale = 1.0;
    std::shared_ptr<chrono::ChBodyAuxRef> m_base;
    std::shared_ptr<chrono::ChBodyAuxRef> m_shoulder;
    std::shared_ptr<chrono::ChBodyAuxRef> m_biceps;
    std::shared_ptr<chrono::ChBodyAuxRef> m_elbow;
    std::shared_ptr<chrono::ChBodyAuxRef> m_wrist;
    std::shared_ptr<chrono::ChBodyAuxRef> m_end_effector;
    std::shared_ptr<chrono::ChBodyAuxRef> m_finger_1;
    std::shared_ptr<chrono::ChBodyAuxRef> m_finger_2;

    std::shared_ptr<chrono::ChLinkLockLock> m_chassis_lock;
    std::shared_ptr<chrono::ChLinkLockLock> m_wrist_lock;
    std::shared_ptr<chrono::ChLinkMotorRotationAngle> m_motor_base_shoulder;
    std::shared_ptr<chrono::ChLinkMotorRotationAngle> m_motor_shoulder_biceps;
    std::shared_ptr<chrono::ChLinkMotorRotationAngle> m_motor_biceps_elbow;
    std::shared_ptr<chrono::ChLinkMotorRotationAngle> m_motor_elbow_effector;
    std::shared_ptr<chrono::ChLinkMotorLinearPosition> m_motor_finger_1;
    std::shared_ptr<chrono::ChLinkMotorLinearPosition> m_motor_finger_2;

    Phase m_phase = Phase::IDLE;
    ArmStatusSnapshot m_status;
    std::shared_ptr<chrono::ChBodyAuxRef> m_target_rock;
    std::shared_ptr<chrono::ChLinkLockLock> m_rock_lock;
    chrono::ChVector3d m_grab_target_world;
    chrono::ChVector3d m_place_target_world;
    std::array<double, 4> m_grab_theta = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> m_place_theta = {0.0, 0.0, 0.0, 0.0};
    double m_start_time = 0.0;
    double m_phase_time = 0.0;
    double m_next_tick = 0.0;
    double m_close_pos = 0.0;
    double m_lift_angle = 0.0;
    bool m_bent_arm = false;
    bool m_contact_seen = false;
};

}  // namespace amd_uw
