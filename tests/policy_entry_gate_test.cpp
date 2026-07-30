#include "policy_entry_gate.h"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

PolicyEntryGateConfig MiniCheetahEntryGateConfig() {
    PolicyEntryGateConfig config;
    config.default_joint_pos = types::Vec3f(0.0f, -0.8f, 1.6f).replicate(4, 1);
    config.max_joint_pos_error = 0.15f;
    config.max_joint_vel_norm = 0.25f;
    config.max_roll_rad = 10.0f * static_cast<float>(M_PI) / 180.0f;
    config.max_pitch_rad = 10.0f * static_cast<float>(M_PI) / 180.0f;
    config.max_base_lin_vel_norm = 0.15f;
    config.target_base_height = 0.28f;
    config.max_base_height_error = 0.05f;
    config.max_control_dt = 0.003f;
    return config;
}

PolicyEntryGateState ReadyState() {
    PolicyEntryGateState state;
    state.joint_pos = types::Vec3f(0.01f, -0.79f, 1.59f).replicate(4, 1);
    state.joint_vel = types::VecXf::Zero(12);
    state.base_rpy = types::Vec3f(0.01f, -0.02f, 1.0f);
    state.base_lin_vel = types::Vec3f(0.02f, -0.01f, 0.0f);
    state.base_height = 0.28f;
    state.control_dt = 0.001f;
    return state;
}

void PolicyEntryGateAllowsStateNearPolicyInitialState() {
    const PolicyEntryGateResult result = EvaluatePolicyEntryGate(
        MiniCheetahEntryGateConfig(),
        ReadyState());

    Expect(result.allowed, "state near policy initial state should be allowed");
    Expect(result.reason == PolicyEntryGateReason::kReady, "allowed state should report ready");
}

void PolicyEntryGateRejectsUnsafeBaseHeight() {
    PolicyEntryGateState state = ReadyState();
    state.base_height = 0.16f;

    const PolicyEntryGateResult result = EvaluatePolicyEntryGate(
        MiniCheetahEntryGateConfig(),
        state);

    Expect(!result.allowed, "unsafe base height should be rejected");
    Expect(result.reason == PolicyEntryGateReason::kBaseHeightUnsafe,
           "unsafe base height should report base height reason");
}

void PolicyEntryGateRejectsTimingThatIsNotReady() {
    PolicyEntryGateState state = ReadyState();
    state.control_dt = 0.010f;

    const PolicyEntryGateResult result = EvaluatePolicyEntryGate(
        MiniCheetahEntryGateConfig(),
        state);

    Expect(!result.allowed, "slow control timing should be rejected");
    Expect(result.reason == PolicyEntryGateReason::kPolicyTimingNotReady,
           "slow control timing should report timing reason");
}

void PolicyEntryGateRejectsWrongSizedJointVelocity() {
    PolicyEntryGateState state = ReadyState();
    state.joint_vel = types::VecXf::Zero(11);

    const PolicyEntryGateResult result = EvaluatePolicyEntryGate(
        MiniCheetahEntryGateConfig(),
        state);

    Expect(!result.allowed, "wrong-sized joint velocity should be rejected");
    Expect(result.reason == PolicyEntryGateReason::kJointVelocityTooHigh,
           "wrong-sized joint velocity should report joint velocity reason");
}

}  // namespace

int main() {
    try {
        PolicyEntryGateAllowsStateNearPolicyInitialState();
        PolicyEntryGateRejectsUnsafeBaseHeight();
        PolicyEntryGateRejectsTimingThatIsNotReady();
        PolicyEntryGateRejectsWrongSizedJointVelocity();
    } catch (const std::exception& e) {
        std::cerr << "policy_entry_gate_test failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
