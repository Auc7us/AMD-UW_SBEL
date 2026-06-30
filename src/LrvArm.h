#pragma once

#include <memory>
#include <string>

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystem.h"

namespace amd_uw {

class LrvArm {
  public:
    LrvArm(chrono::ChSystem* system,
           std::shared_ptr<chrono::ChBody> chassis_body,
           const std::string& amd_uw_data_path,
           const chrono::ChVector3d& mount_pos,
           const chrono::ChQuaternion<>& mount_rot);

  private:
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
};

}  // namespace amd_uw
