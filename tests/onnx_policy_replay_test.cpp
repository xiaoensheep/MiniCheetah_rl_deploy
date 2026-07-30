#include "mini_cheetah_policy_runner_onnx.h"
#include "policy_metadata.h"

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

bool IsFinite(const types::VecXf& values) {
    for (int i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values(i))) {
            return false;
        }
    }
    return true;
}

void OnnxPolicyRunnerReplaysLoggedObservationShape() {
    const PolicyMetadata metadata = LoadPolicyMetadata(ResolvePolicyMetadataPath());
    MiniCheetahPolicyRunnerONNX runner("onnx_replay_test");

    const types::VecXf observation = types::VecXf::Zero(metadata.obs_dim);
    const ReplayPolicyOutput output = runner.ReplayObservation(observation);

    Expect(output.raw_action.size() == metadata.action_dim, "raw action dimension");
    Expect(output.clipped_action.size() == metadata.action_dim, "clipped action dimension");
    Expect(output.target_joint_pos.size() == metadata.action_dim, "target joint position dimension");
    Expect(IsFinite(output.raw_action), "raw action finite");
    Expect(IsFinite(output.clipped_action), "clipped action finite");
    Expect(IsFinite(output.target_joint_pos), "target joint position finite");

    for (int i = 0; i < output.clipped_action.size(); ++i) {
        Expect(output.clipped_action(i) >= -1.000001f, "clipped action lower limit");
        Expect(output.clipped_action(i) <= 1.000001f, "clipped action upper limit");
    }
}

void OnnxPolicyRunnerRejectsWrongObservationShape() {
    MiniCheetahPolicyRunnerONNX runner("onnx_replay_shape_test");

    bool threw = false;
    try {
        (void)runner.ReplayObservation(types::VecXf::Zero(1));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    Expect(threw, "wrong observation shape should fail");
}

}  // namespace

int main() {
    try {
        OnnxPolicyRunnerReplaysLoggedObservationShape();
        OnnxPolicyRunnerRejectsWrongObservationShape();
    } catch (const std::exception& e) {
        std::cerr << "onnx_policy_replay_test failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
