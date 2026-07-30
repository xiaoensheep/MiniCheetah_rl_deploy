#include "episode_log.h"
#include "offline_replay.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr int kPolicyObservationDim = 48;
constexpr int kPolicyActionDim = 12;

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

EpisodeLogRecord MakeRecord(float value) {
    EpisodeLogRecord record;
    record.timestamp = value;
    record.state_machine_state = "rl_control";
    record.observation = types::VecXf::Constant(kPolicyObservationDim, value);
    record.raw_action = types::VecXf::Constant(kPolicyActionDim, value + 0.1f);
    record.clipped_action = types::VecXf::Constant(kPolicyActionDim, value + 0.2f);
    record.target_joint_pos = types::VecXf::Constant(kPolicyActionDim, value + 0.3f);
    record.joint_pos = types::VecXf::Zero(kPolicyActionDim);
    record.joint_vel = types::VecXf::Zero(kPolicyActionDim);
    record.joint_tau = types::VecXf::Zero(kPolicyActionDim);
    record.policy_entry_gate_passed = true;
    record.control_dt = 0.02f;
    return record;
}

class EchoReplayPolicy : public ObservationPolicyReplay {
public:
    explicit EchoReplayPolicy(float offset = 0.0f) : offset_(offset) {}

    ReplayPolicyOutput ReplayObservation(const types::VecXf& observation) override {
        const float value = observation(0) + offset_;
        ReplayPolicyOutput output;
        output.raw_action = types::VecXf::Constant(kPolicyActionDim, value + 0.1f);
        output.clipped_action = types::VecXf::Constant(kPolicyActionDim, value + 0.2f);
        output.target_joint_pos = types::VecXf::Constant(kPolicyActionDim, value + 0.3f);
        return output;
    }

private:
    float offset_;
};

void OfflineReplayPassesMatchingRecords() {
    std::vector<EpisodeLogRecord> records;
    records.push_back(MakeRecord(1.0f));
    records.push_back(MakeRecord(2.0f));
    EchoReplayPolicy policy;

    const OfflineReplaySummary summary = ReplayEpisodeLog(records, policy, OfflineReplayTolerance{});

    Expect(summary.record_count == 2, "record_count");
    Expect(summary.matched_count == 2, "matched_count");
    Expect(summary.mismatch_count == 0, "mismatch_count");
    Expect(summary.all_within_tolerance, "all_within_tolerance");
    ExpectNear(summary.max_raw_action_error, 0.0f, "max raw error");
    ExpectNear(summary.max_clipped_action_error, 0.0f, "max clipped error");
    ExpectNear(summary.max_target_joint_pos_error, 0.0f, "max target error");
}

void OfflineReplayReportsActionMismatch() {
    std::vector<EpisodeLogRecord> records;
    records.push_back(MakeRecord(1.0f));
    EchoReplayPolicy policy(0.05f);

    OfflineReplayTolerance tolerance;
    tolerance.raw_action = 0.01f;
    tolerance.clipped_action = 0.01f;
    tolerance.target_joint_pos = 0.01f;

    const OfflineReplaySummary summary = ReplayEpisodeLog(records, policy, tolerance);

    Expect(summary.record_count == 1, "record_count mismatch case");
    Expect(summary.matched_count == 0, "matched_count mismatch case");
    Expect(summary.mismatch_count == 1, "mismatch_count mismatch case");
    Expect(!summary.all_within_tolerance, "all_within_tolerance mismatch case");
    ExpectNear(summary.max_raw_action_error, 0.05f, "raw mismatch");
    ExpectNear(summary.max_clipped_action_error, 0.05f, "clipped mismatch");
    ExpectNear(summary.max_target_joint_pos_error, 0.05f, "target mismatch");
}

void OfflineReplayReadsEpisodeLogFile() {
    const std::filesystem::path path = "/tmp/mini_cheetah_offline_replay_test.jsonl";
    std::filesystem::remove(path);

    EpisodeLogger logger(path.string());
    logger.Append(MakeRecord(3.0f));
    logger.Append(MakeRecord(4.0f));
    EchoReplayPolicy policy;

    const OfflineReplaySummary summary = ReplayEpisodeLogFile(path.string(), policy, OfflineReplayTolerance{});

    Expect(summary.record_count == 2, "file record_count");
    Expect(summary.matched_count == 2, "file matched_count");
    Expect(summary.mismatch_count == 0, "file mismatch_count");
}

}  // namespace

int main() {
    try {
        OfflineReplayPassesMatchingRecords();
        OfflineReplayReportsActionMismatch();
        OfflineReplayReadsEpisodeLogFile();
    } catch (const std::exception& e) {
        std::cerr << "offline_replay_test failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
