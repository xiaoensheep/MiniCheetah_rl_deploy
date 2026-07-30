#include "simulation_wait_pose.h"

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

    for (int leg = 0; leg < 4; ++leg) {
        ExpectNear(joint_pos(3 * leg), 0.0f, "hip crouch position");
        ExpectNear(joint_pos(3 * leg + 1), -1.45f, "thigh crouch position");
        ExpectNear(joint_pos(3 * leg + 2), 2.35f, "calf crouch position");
    }
}

void CrouchHoldCommandUsesPdPositionHold() {
    const types::Vec3f kp(40.0f, 41.0f, 42.0f);
    const types::Vec3f kd(1.0f, 1.1f, 1.2f);
    const types::MatXf command = simulation_wait_pose::BuildMiniCheetahCrouchHoldCommand(kp, kd);

    if (command.rows() != 12 || command.cols() != 5) {
        throw std::runtime_error("crouch hold command shape");
    }
    for (int leg = 0; leg < 4; ++leg) {
        ExpectNear(command(3 * leg, 0), kp(0), "hip kp");
        ExpectNear(command(3 * leg + 1, 0), kp(1), "thigh kp");
        ExpectNear(command(3 * leg + 2, 0), kp(2), "calf kp");
        ExpectNear(command(3 * leg, 1), 0.0f, "hip q_des");
        ExpectNear(command(3 * leg + 1, 1), -1.45f, "thigh q_des");
        ExpectNear(command(3 * leg + 2, 1), 2.35f, "calf q_des");
        ExpectNear(command(3 * leg, 2), kd(0), "hip kd");
        ExpectNear(command(3 * leg + 1, 2), kd(1), "thigh kd");
        ExpectNear(command(3 * leg + 2, 2), kd(2), "calf kd");
    }
    if (command.col(3).norm() > 1e-6f || command.col(4).norm() > 1e-6f) {
        throw std::runtime_error("crouch hold command velocity and torque feedforward");
    }
}

}  // namespace

int main() {
    try {
        CrouchPoseUsesCanonicalMiniCheetahOrder();
        CrouchHoldCommandUsesPdPositionHold();
    } catch (const std::exception& e) {
        std::cerr << "simulation_wait_pose_test failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
