#include "episode_log_builder.h"

EpisodeLogRecord BuildEpisodeLogRecord(const types::RobotBasicState& robot_state,
                                       const types::VecXf& observation,
                                       const EpisodeLogContext& context) {
    EpisodeLogRecord record;
    record.timestamp = context.timestamp;
    record.state_machine_state = context.state_machine_state;
    record.user_command = types::Vec3f(context.user_command.forward_vel_scale,
                                       context.user_command.side_vel_scale,
                                       context.user_command.turnning_vel_scale);
    record.observation = observation;
    record.raw_action = context.policy_output.raw_action;
    record.clipped_action = context.policy_output.clipped_action;
    record.target_joint_pos = context.commanded_target_joint_pos;
    record.joint_pos = robot_state.joint_pos;
    record.joint_vel = robot_state.joint_vel;
    record.joint_tau = robot_state.joint_tau;
    record.imu_rpy = robot_state.base_rpy;
    record.imu_omega = robot_state.base_omega;
    record.imu_acc = robot_state.base_acc;
    record.base_lin_vel_body = robot_state.base_lin_vel;
    record.policy_entry_gate_passed = context.policy_entry_gate_passed;
    record.policy_entry_gate_reason = context.policy_entry_gate_reason;
    record.clamp_applied = context.clamp_applied;
    record.clamp_reason = context.clamp_reason;
    record.policy_inference_ms = context.policy_inference_ms;
    record.control_dt = context.control_dt;
    return record;
}
