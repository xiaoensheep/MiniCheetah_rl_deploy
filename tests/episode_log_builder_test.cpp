#include "episode_log_builder.h"

#include <cmath>
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

void ExpectNear(float actual, float expected, const std::string& message) {
    if (std::fabs(actual - expected) > 1e-6f) {
        throw std::runtime_error(message);
    }
}

types::RobotBasicState MakeRobotState() {
    types::RobotBasicState state;
    state.base_rpy = types::Vec3f(0.1f, 0.2f, 0.3f);
    state.base_omega = types::Vec3f(1.1f, 1.2f, 1.3f);
    state.base_acc = types::Vec3f(2.1f, 2.2f, 2.3f);
    state.base_lin_vel = types::Vec3f(3.1f, 3.2f, 3.3f);
    state.joint_pos = types::VecXf::LinSpaced(12, 0.0f, 1.1f);
    state.joint_vel = types::VecXf::LinSpaced(12, 2.0f, 3.1f);
    state.joint_tau = types::VecXf::LinSpaced(12, -1.0f, 0.1f);
    return state;
}

EpisodeLogContext MakeContext() {
    EpisodeLogContext context;
    context.timestamp = 12.5;
    context.state_machine_state = "rl_control";
    context.user_command.forward_vel_scale = 0.4f;
    context.user_command.side_vel_scale = -0.2f;
    context.user_command.turnning_vel_scale = 0.1f;
    context.policy_output.raw_action = types::VecXf::Constant(12, 0.8f);
    context.policy_output.clipped_action = types::VecXf::Constant(12, 0.5f);
    context.policy_output.target_joint_pos = types::VecXf::LinSpaced(12, -0.6f, 0.5f);
    context.commanded_target_joint_pos = types::VecXf::LinSpaced(12, -0.3f, 0.8f);
    context.policy_entry_gate_passed = true;
    context.policy_entry_gate_reason = "running";
    context.clamp_applied = false;
    context.clamp_reason = "none";
    context.policy_inference_ms = 0.33f;
    context.control_dt = 0.02f;
    return context;
}

void EpisodeLogBuilderCopiesDeploymentFields() {
    const types::RobotBasicState state = MakeRobotState();
    const types::VecXf observation = types::VecXf::LinSpaced(48, -1.0f, 1.0f);
    const EpisodeLogContext context = MakeContext();

    const EpisodeLogRecord record = BuildEpisodeLogRecord(state, observation, context);

    ExpectNear(static_cast<float>(record.timestamp), 12.5f, "timestamp");
    Expect(record.state_machine_state == "rl_control", "state_machine_state");
    ExpectNear(record.user_command(0), 0.4f, "forward command");
    ExpectNear(record.user_command(1), -0.2f, "side command");
    ExpectNear(record.user_command(2), 0.1f, "turn command");
    Expect(record.observation.size() == 48, "observation dimension");
    ExpectNear(record.observation(47), 1.0f, "observation tail");
    ExpectNear(record.raw_action(0), 0.8f, "raw action");
    ExpectNear(record.clipped_action(0), 0.5f, "clipped action");
    ExpectNear(record.target_joint_pos(11), 0.8f, "target joint position");
    ExpectNear(record.joint_pos(11), 1.1f, "joint position");
    ExpectNear(record.joint_vel(11), 3.1f, "joint velocity");
    ExpectNear(record.joint_tau(0), -1.0f, "joint torque");
    ExpectNear(record.imu_rpy(2), 0.3f, "imu rpy");
    ExpectNear(record.imu_omega(1), 1.2f, "imu omega");
    ExpectNear(record.imu_acc(0), 2.1f, "imu acc");
    ExpectNear(record.base_lin_vel_body(2), 3.3f, "base linear velocity");
    Expect(record.policy_entry_gate_passed, "policy entry gate");
    Expect(record.policy_entry_gate_reason == "running", "policy entry gate reason");
    Expect(!record.clamp_applied, "clamp applied");
    Expect(record.clamp_reason == "none", "clamp reason");
    ExpectNear(record.policy_inference_ms, 0.33f, "policy inference time");
    ExpectNear(record.control_dt, 0.02f, "control dt");
}

}  // namespace

int main() {
    try {
        EpisodeLogBuilderCopiesDeploymentFields();
    } catch (const std::exception& e) {
        std::cerr << "episode_log_builder_test failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
