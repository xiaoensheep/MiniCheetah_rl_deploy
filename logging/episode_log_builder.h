#pragma once

#include "common_types.h"
#include "episode_log.h"
#include "policy_replay.h"

#include <string>

struct EpisodeLogContext {
    double timestamp = 0.0;
    std::string state_machine_state;
    types::UserCommand user_command{};
    ReplayPolicyOutput policy_output;
    types::VecXf commanded_target_joint_pos;
    bool policy_entry_gate_passed = true;
    std::string policy_entry_gate_reason = "running";
    bool clamp_applied = false;
    std::string clamp_reason = "none";
    float policy_inference_ms = 0.0f;
    float control_dt = 0.0f;
};

EpisodeLogRecord BuildEpisodeLogRecord(const types::RobotBasicState& robot_state,
                                       const types::VecXf& observation,
                                       const EpisodeLogContext& context);
