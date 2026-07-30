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
        {"FR_hip_joint", -0.12319f},
        {"FR_thigh_joint", -1.54732f},
        {"FR_calf_joint", 2.60066f},
        {"FL_hip_joint", 0.116436f},
        {"FL_thigh_joint", -1.5563f},
        {"FL_calf_joint", 2.61816f},
        {"RR_hip_joint", -0.0559533f},
        {"RR_thigh_joint", -1.50758f},
        {"RR_calf_joint", 2.46631f},
        {"RL_hip_joint", 0.0480561f},
        {"RL_thigh_joint", -1.5152f},
        {"RL_calf_joint", 2.48509f},
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

inline const std::array<NamedJointPosition, 12>& MiniCheetahCrouchHoldJointPoseEntries() {
    static const std::array<NamedJointPosition, 12> joint_pose = {{
        {"FR_hip_joint", -0.12319f},
        {"FR_thigh_joint", -1.54732f},
        {"FR_calf_joint", 2.35f},
        {"FL_hip_joint", 0.116436f},
        {"FL_thigh_joint", -1.5563f},
        {"FL_calf_joint", 2.35f},
        {"RR_hip_joint", -0.0559533f},
        {"RR_thigh_joint", -1.50758f},
        {"RR_calf_joint", 2.46631f},
        {"RL_hip_joint", 0.0480561f},
        {"RL_thigh_joint", -1.5152f},
        {"RL_calf_joint", 2.48509f},
    }};
    return joint_pose;
}

inline types::VecXf MiniCheetahCrouchHoldJointPos() {
    types::VecXf joint_pos(12);
    const auto& joint_pose = MiniCheetahCrouchHoldJointPoseEntries();
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
    command.col(1) = MiniCheetahCrouchHoldJointPos();
    command.col(2) = joint_kd.replicate(4, 1);
    return command;
}

}  // namespace simulation_wait_pose
