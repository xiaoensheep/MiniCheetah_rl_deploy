#pragma once

#include "common_types.h"

#include <array>

namespace simulation_wait_pose {

struct NamedJointPosition {
    const char* joint_name;
    float position;
};

inline const std::array<NamedJointPosition, 12>& MiniCheetahCrouchJointPoseEntries() {
    static const std::array<NamedJointPosition, 12> joint_pose = {{
        {"FR_hip_joint", 0.0f},
        {"FR_thigh_joint", -1.45f},
        {"FR_calf_joint", 2.35f},
        {"FL_hip_joint", 0.0f},
        {"FL_thigh_joint", -1.45f},
        {"FL_calf_joint", 2.35f},
        {"RR_hip_joint", 0.0f},
        {"RR_thigh_joint", -1.45f},
        {"RR_calf_joint", 2.35f},
        {"RL_hip_joint", 0.0f},
        {"RL_thigh_joint", -1.45f},
        {"RL_calf_joint", 2.35f},
    }};
    return joint_pose;
}

inline types::VecXf MiniCheetahCrouchJointPos() {
    types::VecXf joint_pos(12);
    const auto& joint_pose = MiniCheetahCrouchJointPoseEntries();
    for (std::size_t i = 0; i < joint_pose.size(); ++i) {
        joint_pos(static_cast<int>(i)) = joint_pose[i].position;
    }
    return joint_pos;
}

inline types::MatXf BuildMiniCheetahCrouchHoldCommand(
    const types::Vec3f& joint_kp,
    const types::Vec3f& joint_kd) {
    types::MatXf command = types::MatXf::Zero(12, 5);
    command.col(0) = joint_kp.replicate(4, 1);
    command.col(1) = MiniCheetahCrouchJointPos();
    command.col(2) = joint_kd.replicate(4, 1);
    return command;
}

}  // namespace simulation_wait_pose
