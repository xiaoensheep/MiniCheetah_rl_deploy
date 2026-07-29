#pragma once

#include <string>
#include <vector>

struct PolicyMetadata {
    int obs_dim = 0;
    int action_dim = 0;
    std::string action_semantics;
    std::vector<std::string> joint_order;
    std::vector<float> default_joint_pos;
    std::vector<float> action_scale;
    float lin_vel_scale = 0.0f;
    float omega_scale = 0.0f;
    float dof_vel_scale = 0.0f;
    int decimation = 0;
    float policy_frequency_hz = 0.0f;
    float pd_update_frequency_hz = 0.0f;
};

PolicyMetadata LoadPolicyMetadata(const std::string& path);
