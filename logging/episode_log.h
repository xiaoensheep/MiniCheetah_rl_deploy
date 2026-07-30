#pragma once

#include "common_types.h"

#include <string>
#include <vector>

struct EpisodeLogRecord {
    double timestamp = 0.0;
    std::string state_machine_state;
    types::Vec3f user_command = types::Vec3f::Zero();
    types::VecXf observation;
    types::VecXf raw_action;
    types::VecXf clipped_action;
    types::VecXf target_joint_pos;
    types::VecXf joint_pos;
    types::VecXf joint_vel;
    types::VecXf joint_tau;
    types::Vec3f imu_rpy = types::Vec3f::Zero();
    types::Vec3f imu_omega = types::Vec3f::Zero();
    types::Vec3f imu_acc = types::Vec3f::Zero();
    types::Vec3f base_lin_vel_body = types::Vec3f::Zero();
    bool policy_entry_gate_passed = false;
    std::string policy_entry_gate_reason;
    bool clamp_applied = false;
    std::string clamp_reason;
    float policy_inference_ms = 0.0f;
    float control_dt = 0.0f;
};

std::string SerializeEpisodeLogRecord(const EpisodeLogRecord& record);
EpisodeLogRecord ParseEpisodeLogRecord(const std::string& line);

class EpisodeLogger {
public:
    explicit EpisodeLogger(std::string path);
    void Append(const EpisodeLogRecord& record) const;

private:
    std::string path_;
};

std::vector<EpisodeLogRecord> ReadEpisodeLog(const std::string& path);
