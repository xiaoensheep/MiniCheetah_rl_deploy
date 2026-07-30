#include "offline_replay.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

float MaxAbsError(const types::VecXf& expected, const types::VecXf& actual, const std::string& field) {
    if (expected.size() != actual.size()) {
        throw std::runtime_error("Offline replay vector size mismatch for " + field);
    }
    if (expected.size() == 0) {
        return 0.0f;
    }
    return (expected - actual).cwiseAbs().maxCoeff();
}

bool WithinTolerance(const OfflineReplayRecordResult& result, const OfflineReplayTolerance& tolerance) {
    return result.raw_action_error <= tolerance.raw_action &&
           result.clipped_action_error <= tolerance.clipped_action &&
           result.target_joint_pos_error <= tolerance.target_joint_pos;
}

void UpdateSummary(OfflineReplaySummary* summary, const OfflineReplayRecordResult& result) {
    summary->max_raw_action_error = std::max(summary->max_raw_action_error, result.raw_action_error);
    summary->max_clipped_action_error = std::max(summary->max_clipped_action_error, result.clipped_action_error);
    summary->max_target_joint_pos_error = std::max(summary->max_target_joint_pos_error, result.target_joint_pos_error);
    if (result.within_tolerance) {
        ++summary->matched_count;
    } else {
        ++summary->mismatch_count;
        summary->all_within_tolerance = false;
    }
    summary->records.push_back(result);
}

}  // namespace

OfflineReplaySummary ReplayEpisodeLog(const std::vector<EpisodeLogRecord>& records,
                                      ObservationPolicyReplay& policy,
                                      const OfflineReplayTolerance& tolerance) {
    OfflineReplaySummary summary;
    summary.record_count = records.size();
    summary.records.reserve(records.size());

    for (const EpisodeLogRecord& record : records) {
        const ReplayPolicyOutput output = policy.ReplayObservation(record.observation);

        OfflineReplayRecordResult result;
        result.timestamp = record.timestamp;
        result.raw_action_error = MaxAbsError(record.raw_action, output.raw_action, "raw_action");
        result.clipped_action_error = MaxAbsError(record.clipped_action, output.clipped_action, "clipped_action");
        result.target_joint_pos_error =
            MaxAbsError(record.target_joint_pos, output.target_joint_pos, "target_joint_pos");
        result.within_tolerance = WithinTolerance(result, tolerance);

        UpdateSummary(&summary, result);
    }

    return summary;
}

OfflineReplaySummary ReplayEpisodeLogFile(const std::string& path,
                                          ObservationPolicyReplay& policy,
                                          const OfflineReplayTolerance& tolerance) {
    return ReplayEpisodeLog(ReadEpisodeLog(path), policy, tolerance);
}
