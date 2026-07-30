#pragma once

#include "common_types.h"

struct ReplayPolicyOutput {
    types::VecXf raw_action;
    types::VecXf clipped_action;
    types::VecXf target_joint_pos;
};

class ObservationPolicyReplay {
public:
    virtual ~ObservationPolicyReplay() = default;
    virtual ReplayPolicyOutput ReplayObservation(const types::VecXf& observation) = 0;
};
