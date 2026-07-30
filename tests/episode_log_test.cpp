#include "episode_log.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
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

void ReplaceJsonArrayField(std::string* line, const std::string& key, const std::string& replacement) {
    const std::string prefix = "\"" + key + "\":[";
    const std::size_t start = line->find(prefix);
    Expect(start != std::string::npos, key + " field");
    const std::size_t end = line->find(']', start);
    Expect(end != std::string::npos, key + " array end");
    line->replace(start, end - start + 1, "\"" + key + "\":" + replacement);
}

EpisodeLogRecord MakeCompleteRecord() {
    EpisodeLogRecord record;
    record.timestamp = 1.25;
    record.state_machine_state = "rl_control";
    record.user_command = types::Vec3f(0.1f, -0.2f, 0.3f);
    record.observation = types::VecXf::LinSpaced(48, 0.0f, 4.7f);
    record.raw_action = types::VecXf::Constant(12, 0.8f);
    record.clipped_action = types::VecXf::Constant(12, 0.5f);
    record.target_joint_pos = types::VecXf::LinSpaced(12, -0.5f, 0.6f);
    record.joint_pos = types::VecXf::LinSpaced(12, 0.0f, 1.1f);
    record.joint_vel = types::VecXf::LinSpaced(12, 1.0f, 2.1f);
    record.joint_tau = types::VecXf::LinSpaced(12, -1.0f, 0.1f);
    record.imu_rpy = types::Vec3f(0.01f, -0.02f, 0.03f);
    record.imu_omega = types::Vec3f(0.4f, -0.5f, 0.6f);
    record.imu_acc = types::Vec3f(0.0f, 0.0f, 9.815f);
    record.base_lin_vel_body = types::Vec3f(0.12f, -0.23f, 0.34f);
    record.policy_entry_gate_passed = true;
    record.policy_entry_gate_reason = "ready";
    record.clamp_applied = true;
    record.clamp_reason = "none";
    record.policy_inference_ms = 0.42f;
    record.control_dt = 0.001f;
    return record;
}

void EpisodeLogRecordRoundTripsThroughJsonLine() {
    const EpisodeLogRecord record = MakeCompleteRecord();

    const std::string line = SerializeEpisodeLogRecord(record);
    const EpisodeLogRecord parsed = ParseEpisodeLogRecord(line);

    ExpectNear(static_cast<float>(parsed.timestamp), 1.25f, "timestamp");
    Expect(parsed.state_machine_state == "rl_control", "state_machine_state");
    ExpectNear(parsed.user_command(1), -0.2f, "user_command");
    Expect(parsed.observation.size() == 48, "observation size");
    ExpectNear(parsed.observation(47), 4.7f, "observation[47]");
    Expect(parsed.raw_action.size() == 12, "raw_action size");
    ExpectNear(parsed.raw_action(0), 0.8f, "raw_action[0]");
    ExpectNear(parsed.clipped_action(0), 0.5f, "clipped_action[0]");
    ExpectNear(parsed.target_joint_pos(11), 0.6f, "target_joint_pos[11]");
    ExpectNear(parsed.joint_pos(11), 1.1f, "joint_pos[11]");
    ExpectNear(parsed.joint_vel(11), 2.1f, "joint_vel[11]");
    ExpectNear(parsed.joint_tau(0), -1.0f, "joint_tau[0]");
    ExpectNear(parsed.imu_rpy(2), 0.03f, "imu_rpy yaw");
    ExpectNear(parsed.imu_omega(1), -0.5f, "imu_omega y");
    ExpectNear(parsed.imu_acc(2), 9.815f, "imu_acc z");
    ExpectNear(parsed.base_lin_vel_body(0), 0.12f, "base_lin_vel_body x");
    Expect(parsed.policy_entry_gate_passed, "policy_entry_gate_passed");
    Expect(parsed.policy_entry_gate_reason == "ready", "policy_entry_gate_reason");
    Expect(parsed.clamp_applied, "clamp_applied");
    Expect(parsed.clamp_reason == "none", "clamp_reason");
    ExpectNear(parsed.policy_inference_ms, 0.42f, "policy_inference_ms");
    ExpectNear(parsed.control_dt, 0.001f, "control_dt");
}

void EpisodeLogEscapesStringsAsSingleLineJson() {
    EpisodeLogRecord record = MakeCompleteRecord();
    record.policy_entry_gate_reason = "ready\nwith\ttab";
    record.clamp_reason = "quoted \"slash\\";

    const std::string line = SerializeEpisodeLogRecord(record);
    Expect(line.find('\n') == std::string::npos, "json line contains raw newline");

    const EpisodeLogRecord parsed = ParseEpisodeLogRecord(line);
    Expect(parsed.policy_entry_gate_reason == "ready\nwith\ttab", "escaped gate reason");
    Expect(parsed.clamp_reason == "quoted \"slash\\", "escaped clamp reason");
}

void EpisodeLogRejectsMalformedNumericArrays() {
    std::string line = SerializeEpisodeLogRecord(MakeCompleteRecord());
    ReplaceJsonArrayField(&line, "raw_action", "[0.8,garbage,0.8]");

    bool threw = false;
    try {
        (void)ParseEpisodeLogRecord(line);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    Expect(threw, "malformed numeric array should fail");
}

void EpisodeLoggerAppendsJsonLines() {
    const std::filesystem::path path = "/tmp/mini_cheetah_episode_log_test.jsonl";
    std::filesystem::remove(path);

    EpisodeLogger logger(path.string());
    logger.Append(MakeCompleteRecord());
    logger.Append(MakeCompleteRecord());

    const std::vector<EpisodeLogRecord> records = ReadEpisodeLog(path.string());
    Expect(records.size() == 2, "episode log record count");
    ExpectNear(static_cast<float>(records[0].timestamp), 1.25f, "first timestamp");
    ExpectNear(records[1].policy_inference_ms, 0.42f, "second policy inference time");
}

}  // namespace

int main() {
    try {
        EpisodeLogRecordRoundTripsThroughJsonLine();
        EpisodeLogEscapesStringsAsSingleLineJson();
        EpisodeLogRejectsMalformedNumericArrays();
        EpisodeLoggerAppendsJsonLines();
    } catch (const std::exception& e) {
        std::cerr << "episode_log_test failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
