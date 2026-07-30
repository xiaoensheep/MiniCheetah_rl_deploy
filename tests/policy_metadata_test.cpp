#include "policy_metadata.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

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

std::string WriteValidMetadata() {
    const std::string path = "/tmp/mini_cheetah_policy_metadata_valid.json";
    std::ofstream out(path);
    out << R"json({
  "obs_dim": 48,
  "action_dim": 12,
  "action_semantics": "target_joint_position",
  "joint_order": [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
  ],
  "default_joint_pos": [
    0.0, 0.8, -1.6,
    0.0, 0.8, -1.6,
    0.0, 0.8, -1.6,
    0.0, 0.8, -1.6
  ],
  "robot_default_joint_pos": [
    0.0, -0.8, 1.6,
    0.0, -0.8, 1.6,
    0.0, -0.8, 1.6,
    0.0, -0.8, 1.6
  ],
  "joint_position_sign": [
    1.0, -1.0, -1.0,
    1.0, -1.0, -1.0,
    1.0, -1.0, -1.0,
    1.0, -1.0, -1.0
  ],
  "action_scale": [
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25
  ],
  "kp": [
    25.0, 25.0, 25.0,
    25.0, 25.0, 25.0,
    25.0, 25.0, 25.0,
    25.0, 25.0, 25.0
  ],
  "kd": [
    0.5, 0.5, 0.5,
    0.5, 0.5, 0.5,
    0.5, 0.5, 0.5,
    0.5, 0.5, 0.5
  ],
  "action_clip": 100.0,
  "lin_vel_scale": 1.0,
  "omega_scale": 1.0,
  "dof_vel_scale": 1.0,
  "target_base_height": 0.28,
  "decimation": 20,
  "policy_frequency_hz": 50.0,
  "pd_update_frequency_hz": 1000.0
})json";
    return path;
}

std::string WriteMetadataWithIncompleteJointOrder() {
    const std::string path = "/tmp/mini_cheetah_policy_metadata_bad_joint_order.json";
    std::ofstream out(path);
    out << R"json({
  "obs_dim": 48,
  "action_dim": 12,
  "action_semantics": "target_joint_position",
  "joint_order": [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint"
  ],
  "default_joint_pos": [
    0.0, -0.8, 1.6,
    0.0, -0.8, 1.6,
    0.0, -0.8, 1.6,
    0.0, -0.8, 1.6
  ],
  "action_scale": [
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25
  ],
  "lin_vel_scale": 2.0,
  "omega_scale": 0.25,
  "dof_vel_scale": 0.05,
  "decimation": 12,
  "policy_frequency_hz": 50.0,
  "pd_update_frequency_hz": 1000.0
})json";
    return path;
}

std::string WriteMetadataWithDuplicateJointOrder() {
    const std::string path = WriteValidMetadata();
    std::ofstream out(path);
    out << R"json({
  "obs_dim": 48,
  "action_dim": 12,
  "action_semantics": "target_joint_position",
  "joint_order": [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "FR_hip_joint"
  ],
  "default_joint_pos": [
    0.0, 0.8, -1.6,
    0.0, 0.8, -1.6,
    0.0, 0.8, -1.6,
    0.0, 0.8, -1.6
  ],
  "action_scale": [
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25,
    0.25, 0.25, 0.25
  ],
  "lin_vel_scale": 1.0,
  "omega_scale": 1.0,
  "dof_vel_scale": 1.0,
  "decimation": 20,
  "policy_frequency_hz": 50.0,
  "pd_update_frequency_hz": 1000.0
})json";
    return path;
}

void ValidPolicyMetadataLoadsDeploymentContract() {
    const PolicyMetadata metadata = LoadPolicyMetadata(WriteValidMetadata());

    const std::vector<std::string> canonical_joint_order = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

    Expect(metadata.obs_dim == 48, "obs_dim");
    Expect(metadata.action_dim == 12, "action_dim");
    Expect(metadata.action_semantics == "target_joint_position", "action_semantics");
    Expect(metadata.joint_order == canonical_joint_order, "joint_order");
    Expect(metadata.default_joint_pos.size() == 12, "default_joint_pos size");
    Expect(metadata.robot_default_joint_pos.size() == 12, "robot_default_joint_pos size");
    Expect(metadata.joint_position_sign.size() == 12, "joint_position_sign size");
    Expect(metadata.action_scale.size() == 12, "action_scale size");
    Expect(metadata.kp.size() == 12, "kp size");
    Expect(metadata.kd.size() == 12, "kd size");
    ExpectNear(metadata.default_joint_pos[1], 0.8f, "policy default thigh");
    ExpectNear(metadata.default_joint_pos[2], -1.6f, "policy default calf");
    ExpectNear(metadata.robot_default_joint_pos[1], -0.8f, "robot default thigh");
    ExpectNear(metadata.robot_default_joint_pos[2], 1.6f, "robot default calf");
    ExpectNear(metadata.joint_position_sign[1], -1.0f, "thigh sign");
    ExpectNear(metadata.joint_position_sign[2], -1.0f, "calf sign");
    ExpectNear(metadata.action_scale[0], 0.25f, "action_scale[0]");
    ExpectNear(metadata.kp[0], 25.0f, "kp[0]");
    ExpectNear(metadata.kd[0], 0.5f, "kd[0]");
    ExpectNear(metadata.action_clip, 100.0f, "action_clip");
    ExpectNear(metadata.lin_vel_scale, 1.0f, "lin_vel_scale");
    ExpectNear(metadata.omega_scale, 1.0f, "omega_scale");
    ExpectNear(metadata.dof_vel_scale, 1.0f, "dof_vel_scale");
    ExpectNear(metadata.target_base_height, 0.28f, "target_base_height");
    Expect(metadata.decimation == 20, "decimation");
    ExpectNear(metadata.policy_frequency_hz, 50.0f, "policy_frequency_hz");
    ExpectNear(metadata.pd_update_frequency_hz, 1000.0f, "pd_update_frequency_hz");
}

void PolicyMetadataRejectsIncompleteJointOrder() {
    bool rejected = false;
    try {
        LoadPolicyMetadata(WriteMetadataWithIncompleteJointOrder());
    } catch (const std::exception&) {
        rejected = true;
    }

    Expect(rejected, "incomplete joint_order should be rejected");
}

void PolicyMetadataRejectsDuplicateJointOrder() {
    bool rejected = false;
    try {
        LoadPolicyMetadata(WriteMetadataWithDuplicateJointOrder());
    } catch (const std::exception&) {
        rejected = true;
    }

    Expect(rejected, "duplicate joint_order should be rejected");
}

}  // namespace

int main() {
    try {
        ValidPolicyMetadataLoadsDeploymentContract();
        PolicyMetadataRejectsIncompleteJointOrder();
        PolicyMetadataRejectsDuplicateJointOrder();
    } catch (const std::exception& e) {
        std::cerr << "policy_metadata_test failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
