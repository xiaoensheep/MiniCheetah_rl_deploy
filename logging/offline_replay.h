#pragma once

#include "common_types.h"
#include "episode_log.h"
#include "policy_replay.h"

#include <cstddef>
#include <string>
#include <vector>

struct OfflineReplayTolerance {
    float raw_action = 1e-5f;
    float clipped_action = 1e-5f;
    float target_joint_pos = 1e-5f;
};

struct OfflineReplayRecordResult {
    double timestamp = 0.0;
    float raw_action_error = 0.0f;
    float clipped_action_error = 0.0f;
    float target_joint_pos_error = 0.0f;
    bool within_tolerance = true;
};

struct OfflineReplaySummary {
    std::size_t record_count = 0;
    std::size_t matched_count = 0;
    std::size_t mismatch_count = 0;
    float max_raw_action_error = 0.0f;
    float max_clipped_action_error = 0.0f;
    float max_target_joint_pos_error = 0.0f;
    bool all_within_tolerance = true;
    std::vector<OfflineReplayRecordResult> records;
};

OfflineReplaySummary ReplayEpisodeLog(const std::vector<EpisodeLogRecord>& records,
                                      ObservationPolicyReplay& policy,
                                      const OfflineReplayTolerance& tolerance);

OfflineReplaySummary ReplayEpisodeLogFile(const std::string& path,
                                          ObservationPolicyReplay& policy,
                                          const OfflineReplayTolerance& tolerance);
