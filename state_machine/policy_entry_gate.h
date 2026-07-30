#pragma once

#include "common_types.h"

#include <string>

namespace policy_entry_gate {
constexpr float kPi = 3.14159265358979323846f;
}

enum class PolicyEntryGateReason {
    kReady,
    kJointPositionError,
    kJointVelocityTooHigh,
    kBaseOrientationUnsafe,
    kBaseVelocityTooHigh,
    kBaseHeightUnsafe,
    kPolicyTimingNotReady,
};

struct PolicyEntryGateConfig {
    types::VecXf default_joint_pos;
    float max_joint_pos_error = 0.15f;
    float max_joint_vel_norm = 0.25f;
    float max_roll_rad = 10.0f * policy_entry_gate::kPi / 180.0f;
    float max_pitch_rad = 10.0f * policy_entry_gate::kPi / 180.0f;
    float max_base_lin_vel_norm = 0.15f;
    float target_base_height = 0.28f;
    float max_base_height_error = 0.05f;
    float max_control_dt = 0.003f;
};

struct PolicyEntryGateState {
    types::VecXf joint_pos;
    types::VecXf joint_vel;
    types::Vec3f base_rpy = types::Vec3f::Zero();
    types::Vec3f base_lin_vel = types::Vec3f::Zero();
    float base_height = 0.0f;
    float control_dt = 0.0f;
};

struct PolicyEntryGateResult {
    bool allowed = false;
    PolicyEntryGateReason reason = PolicyEntryGateReason::kReady;
};

PolicyEntryGateResult EvaluatePolicyEntryGate(
    const PolicyEntryGateConfig& config,
    const PolicyEntryGateState& state);

std::string PolicyEntryGateReasonToString(PolicyEntryGateReason reason);
