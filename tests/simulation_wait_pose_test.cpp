#include "simulation_wait_pose.h"
#include "simulation_packet_codec.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void ExpectNear(float actual, float expected, const std::string& message) {
    if (std::fabs(actual - expected) > 1e-6f) {
        throw std::runtime_error(message);
    }
}

void ExpectEqual(const std::string& actual, const std::string& expected, const std::string& message) {
    if (actual != expected) {
        throw std::runtime_error(message);
    }
}

void CrouchPoseUsesCanonicalMiniCheetahOrder() {
    const auto& entries = simulation_wait_pose::MiniCheetahCrouchJointPoseEntries();
    const char* expected_names[12] = {
        "FR_hip_joint",
        "FR_thigh_joint",
        "FR_calf_joint",
        "FL_hip_joint",
        "FL_thigh_joint",
        "FL_calf_joint",
        "RR_hip_joint",
        "RR_thigh_joint",
        "RR_calf_joint",
        "RL_hip_joint",
        "RL_thigh_joint",
        "RL_calf_joint",
    };
    for (int i = 0; i < 12; ++i) {
        ExpectEqual(entries[static_cast<std::size_t>(i)].joint_name, expected_names[i], "canonical joint name");
    }

    const types::VecXf joint_pos = simulation_wait_pose::MiniCheetahCrouchJointPos();
    if (joint_pos.size() != 12) {
        throw std::runtime_error("crouch joint pose size");
    }

    const float expected_positions[12] = {
        -0.12319f,
        -1.54732f,
        2.60066f,
        0.116436f,
        -1.5563f,
        2.61816f,
        -0.0559533f,
        -1.50758f,
        2.46631f,
        0.0480561f,
        -1.5152f,
        2.48509f,
    };
    for (int i = 0; i < 12; ++i) {
        ExpectNear(joint_pos(i), expected_positions[i], "crouch keyframe joint position");
    }
}

void CrouchHoldCommandUsesPdPositionHold() {
    const types::Vec3f kp(40.0f, 41.0f, 42.0f);
    const types::Vec3f kd(1.0f, 1.1f, 1.2f);
    const types::MatXf command = simulation_wait_pose::BuildMiniCheetahCrouchHoldCommand(kp, kd);

    if (command.rows() != 12 || command.cols() != 5) {
        throw std::runtime_error("crouch hold command shape");
    }
    const float expected_positions[12] = {
        -0.12319f,
        -1.54732f,
        2.35f,
        0.116436f,
        -1.5563f,
        2.35f,
        -0.0559533f,
        -1.50758f,
        2.46631f,
        0.0480561f,
        -1.5152f,
        2.48509f,
    };
    for (int leg = 0; leg < 4; ++leg) {
        ExpectNear(command(3 * leg, 0), kp(0), "hip kp");
        ExpectNear(command(3 * leg + 1, 0), kp(1), "thigh kp");
        ExpectNear(command(3 * leg + 2, 0), kp(2), "calf kp");
        ExpectNear(command(3 * leg, 2), kd(0), "hip kd");
        ExpectNear(command(3 * leg + 1, 2), kd(1), "thigh kd");
        ExpectNear(command(3 * leg + 2, 2), kd(2), "calf kd");
    }
    for (int i = 0; i < 12; ++i) {
        ExpectNear(command(i, 1), expected_positions[i], "crouch keyframe q_des");
    }
    if (command.col(3).norm() > 1e-6f || command.col(4).norm() > 1e-6f) {
        throw std::runtime_error("crouch hold command velocity and torque feedforward");
    }
}

void CrouchHoldCommandFitsDeploymentLimits() {
    const types::Vec3f kp(40.0f, 40.0f, 40.0f);
    const types::Vec3f kd(1.0f, 1.0f, 1.0f);
    const types::MatXf command = simulation_wait_pose::BuildMiniCheetahCrouchHoldCommand(kp, kd);
    const JointCommandLimitResult limit_result =
        ValidateJointCommandLimits(command, MiniCheetahJointCommandLimits(), 12);
    if (!limit_result.valid) {
        throw std::runtime_error("crouch hold command violates deployment limits");
    }
    (void)EncodeJointCommandPacket(command, 12);
}

}  // namespace

int main() {
    try {
        CrouchPoseUsesCanonicalMiniCheetahOrder();
        CrouchHoldCommandUsesPdPositionHold();
        CrouchHoldCommandFitsDeploymentLimits();
    } catch (const std::exception& e) {
        std::cerr << "simulation_wait_pose_test failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
