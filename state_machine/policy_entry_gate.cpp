#include "policy_entry_gate.h"

#include <cmath>

PolicyEntryGateResult EvaluatePolicyEntryGate(
    const PolicyEntryGateConfig& config,
    const PolicyEntryGateState& state) {
    PolicyEntryGateResult result;

    if (state.joint_pos.size() != config.default_joint_pos.size() ||
        (state.joint_pos - config.default_joint_pos).cwiseAbs().maxCoeff() > config.max_joint_pos_error) {
        result.reason = PolicyEntryGateReason::kJointPositionError;
        return result;
    }

    if (state.joint_vel.size() != config.default_joint_pos.size() ||
        state.joint_vel.norm() > config.max_joint_vel_norm) {
        result.reason = PolicyEntryGateReason::kJointVelocityTooHigh;
        return result;
    }

    if (std::fabs(state.base_rpy(0)) > config.max_roll_rad ||
        std::fabs(state.base_rpy(1)) > config.max_pitch_rad) {
        result.reason = PolicyEntryGateReason::kBaseOrientationUnsafe;
        return result;
    }

    if (state.base_lin_vel.norm() > config.max_base_lin_vel_norm) {
        result.reason = PolicyEntryGateReason::kBaseVelocityTooHigh;
        return result;
    }

    if (!std::isfinite(state.base_height) ||
        std::fabs(state.base_height - config.target_base_height) > config.max_base_height_error) {
        result.reason = PolicyEntryGateReason::kBaseHeightUnsafe;
        return result;
    }

    if (!std::isfinite(state.control_dt) ||
        state.control_dt <= 0.0f ||
        state.control_dt > config.max_control_dt) {
        result.reason = PolicyEntryGateReason::kPolicyTimingNotReady;
        return result;
    }

    result.allowed = true;
    result.reason = PolicyEntryGateReason::kReady;
    return result;
}

std::string PolicyEntryGateReasonToString(PolicyEntryGateReason reason) {
    switch (reason) {
        case PolicyEntryGateReason::kReady:
            return "ready";
        case PolicyEntryGateReason::kJointPositionError:
            return "joint_position_error";
        case PolicyEntryGateReason::kJointVelocityTooHigh:
            return "joint_velocity_too_high";
        case PolicyEntryGateReason::kBaseOrientationUnsafe:
            return "base_orientation_unsafe";
        case PolicyEntryGateReason::kBaseVelocityTooHigh:
            return "base_velocity_too_high";
        case PolicyEntryGateReason::kBaseHeightUnsafe:
            return "base_height_unsafe";
        case PolicyEntryGateReason::kPolicyTimingNotReady:
            return "policy_timing_not_ready";
    }
    return "unknown";
}
