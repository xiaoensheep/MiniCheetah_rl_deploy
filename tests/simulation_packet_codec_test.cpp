#include "simulation_packet_codec.h"

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <limits>
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

std::vector<char> BuildRobotStatePacket(int dof_num) {
    const double timestamp = 1.25;
    std::vector<float> values;
    values.reserve(13 + 3 * dof_num);

    values.push_back(0.28f);                         // meters
    values.insert(values.end(), {0.12f, -0.23f, 0.34f});        // body-frame m/s
    values.insert(values.end(), {0.10f, -0.20f, 0.30f});        // roll, pitch, yaw radians
    values.insert(values.end(), {0.01f, -0.02f, 9.815f});      // body-frame proper acceleration
    values.insert(values.end(), {0.40f, -0.50f, 0.60f});       // body-frame rad/s
    const std::vector<float> q_by_canonical_joint = {
        -0.11f, -0.81f, 1.51f,
         0.12f, -0.82f, 1.52f,
        -0.13f, -0.83f, 1.53f,
         0.14f, -0.84f, 1.54f};
    const std::vector<float> dq_by_canonical_joint = {
        0.01f, 0.02f, 0.03f,
        0.04f, 0.05f, 0.06f,
        0.07f, 0.08f, 0.09f,
        0.10f, 0.11f, 0.12f};
    const std::vector<float> tau_by_canonical_joint = {
        1.1f, 1.2f, 1.3f,
        1.4f, 1.5f, 1.6f,
        1.7f, 1.8f, 1.9f,
        2.0f, 2.1f, 2.2f};
    values.insert(values.end(), q_by_canonical_joint.begin(), q_by_canonical_joint.end());
    values.insert(values.end(), dq_by_canonical_joint.begin(), dq_by_canonical_joint.end());
    values.insert(values.end(), tau_by_canonical_joint.begin(), tau_by_canonical_joint.end());

    std::vector<char> packet(sizeof(double) + sizeof(float) * values.size());
    std::memcpy(packet.data(), &timestamp, sizeof(double));
    std::memcpy(packet.data() + sizeof(double), values.data(), sizeof(float) * values.size());
    return packet;
}

std::vector<char> BuildOldRobotStatePacketWithoutBaseHeight(int dof_num) {
    const double timestamp = 1.25;
    std::vector<float> values;
    values.reserve(12 + 3 * dof_num);

    values.insert(values.end(), {0.1f, -0.2f, 0.3f});
    values.insert(values.end(), {0.01f, -0.02f, 0.03f});
    values.insert(values.end(), {9.1f, 9.2f, 9.3f});
    values.insert(values.end(), {0.4f, -0.5f, 0.6f});
    for (int i = 0; i < dof_num; ++i) values.push_back(1.0f + i);
    for (int i = 0; i < dof_num; ++i) values.push_back(2.0f + i);
    for (int i = 0; i < dof_num; ++i) values.push_back(3.0f + i);

    std::vector<char> packet(sizeof(double) + sizeof(float) * values.size());
    std::memcpy(packet.data(), &timestamp, sizeof(double));
    std::memcpy(packet.data() + sizeof(double), values.data(), sizeof(float) * values.size());
    return packet;
}

void CodecPublishesCanonicalJointOrder() {
    const std::vector<std::string> expected = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

    Expect(CanonicalJointOrder() == expected, "canonical joint order");
}

void RobotStatePacketDecodesDeploymentInterfaceContract() {
    const int dof_num = 12;
    const std::vector<char> packet = BuildRobotStatePacket(dof_num);
    const DecodedRobotStatePacket state = DecodeRobotStatePacket(packet.data(), packet.size(), dof_num);

    ExpectNear(static_cast<float>(state.timestamp), 1.25f, "timestamp");
    ExpectNear(state.base_height, 0.28f, "base_height");
    ExpectNear(state.base_lin_vel_body(0), 0.12f, "body-frame base linear velocity x in m/s");
    ExpectNear(state.base_lin_vel_body(1), -0.23f, "body-frame base linear velocity y in m/s");
    ExpectNear(state.rpy_rad(0), 0.10f, "roll in radians");
    ExpectNear(state.rpy_rad(1), -0.20f, "pitch in radians");
    ExpectNear(state.rpy_rad(2), 0.30f, "yaw in radians");
    ExpectNear(state.proper_acc_body(2), 9.815f, "upright body-frame proper acceleration z");
    ExpectNear(state.omega_body(1), -0.5f, "omega y");
    Expect(state.joint_pos.size() == dof_num, "joint_pos size");
    Expect(state.joint_vel.size() == dof_num, "joint_vel size");
    Expect(state.joint_tau.size() == dof_num, "joint_tau size");
    ExpectNear(state.joint_pos(0), -0.11f, "FR_hip joint_pos");
    ExpectNear(state.joint_pos(1), -0.81f, "FR_thigh joint_pos");
    ExpectNear(state.joint_pos(2), 1.51f, "FR_calf joint_pos");
    ExpectNear(state.joint_pos(9), 0.14f, "RL_hip joint_pos");
    ExpectNear(state.joint_pos(10), -0.84f, "RL_thigh joint_pos");
    ExpectNear(state.joint_pos(11), 1.54f, "RL_calf joint_pos");
    ExpectNear(state.joint_vel(0), 0.01f, "FR_hip joint_vel");
    ExpectNear(state.joint_vel(11), 0.12f, "RL_calf joint_vel");
    ExpectNear(state.joint_tau(0), 1.1f, "FR_hip joint_tau");
    ExpectNear(state.joint_tau(11), 2.2f, "RL_calf joint_tau");
}

void RobotStatePacketRejectsNullPayload() {
    bool rejected = false;
    try {
        DecodeRobotStatePacket(nullptr, RobotStatePacketSize(12), 12);
    } catch (const std::exception&) {
        rejected = true;
    }
    Expect(rejected, "null robot state packet should be rejected");
}

void RobotStatePacketRejectsOldLayoutWithoutBaseHeight() {
    bool rejected = false;
    const std::vector<char> packet = BuildOldRobotStatePacketWithoutBaseHeight(12);
    try {
        DecodeRobotStatePacket(packet.data(), packet.size(), 12);
    } catch (const std::exception&) {
        rejected = true;
    }
    Expect(rejected, "old robot state packet layout should be rejected");
}

void RobotStatePacketRejectsWrongSize() {
    bool rejected = false;
    const std::vector<char> packet = BuildRobotStatePacket(12);
    try {
        DecodeRobotStatePacket(packet.data(), packet.size() - 1, 12);
    } catch (const std::exception&) {
        rejected = true;
    }
    Expect(rejected, "wrong-size robot state packet should be rejected");
}

void RobotStatePacketRejectsNonCanonicalDofCount() {
    bool rejected = false;
    const std::vector<char> packet(sizeof(double) + sizeof(float) * (13 + 3 * 6), 0);
    try {
        DecodeRobotStatePacket(packet.data(), packet.size(), 6);
    } catch (const std::exception&) {
        rejected = true;
    }
    Expect(rejected, "non-canonical dof count should be rejected");
}

void JointCommandPacketEncodesDeploymentInterfaceContractColumns() {
    const int dof_num = 12;
    types::MatXf command(dof_num, kJointCommandColumnCount);
    for (int i = 0; i < dof_num; ++i) {
        command(i, kJointCommandKp) = 10.0f + i;
        command(i, kJointCommandPosition) = 0.01f * static_cast<float>(i);
        command(i, kJointCommandKd) = 1.0f + 0.1f * static_cast<float>(i);
        command(i, kJointCommandVelocity) = 0.2f + 0.01f * static_cast<float>(i);
        command(i, kJointCommandFeedForwardTorque) = 0.3f + 0.01f * static_cast<float>(i);
    }

    const std::vector<char> packet = EncodeJointCommandPacket(command, dof_num);

    Expect(packet.size() == JointCommandPacketSize(dof_num), "joint command packet size");
    std::vector<float> values(5 * dof_num);
    std::memcpy(values.data(), packet.data(), packet.size());

    ExpectNear(values[0], 10.0f, "kp[0]");
    ExpectNear(values[11], 21.0f, "kp[11]");
    ExpectNear(values[12], 0.0f, "q_des[0]");
    ExpectNear(values[23], 0.11f, "q_des[11]");
    ExpectNear(values[24], 1.0f, "kd[0]");
    ExpectNear(values[35], 2.1f, "kd[11]");
    ExpectNear(values[36], 0.2f, "dq_des[0]");
    ExpectNear(values[47], 0.31f, "dq_des[11]");
    ExpectNear(values[48], 0.3f, "tau_ff[0]");
    ExpectNear(values[59], 0.41f, "tau_ff[11]");
}

void JointCommandPacketComputesPdCommandSemantics() {
    types::MatXf command = types::MatXf::Zero(2, kJointCommandColumnCount);
    command(0, kJointCommandKp) = 10.0f;
    command(0, kJointCommandPosition) = 1.5f;
    command(0, kJointCommandKd) = 2.0f;
    command(0, kJointCommandVelocity) = -0.2f;
    command(0, kJointCommandFeedForwardTorque) = 0.3f;
    command(1, kJointCommandKp) = 20.0f;
    command(1, kJointCommandPosition) = -1.0f;
    command(1, kJointCommandKd) = 3.0f;
    command(1, kJointCommandVelocity) = 0.4f;
    command(1, kJointCommandFeedForwardTorque) = -0.5f;

    types::VecXf joint_pos(2);
    joint_pos << 1.0f, -0.5f;
    types::VecXf joint_vel(2);
    joint_vel << 0.1f, -0.2f;

    const types::VecXf torque = ComputePdCommandTorque(command, joint_pos, joint_vel, 2);

    ExpectNear(torque(0), 5.0f + -0.6f + 0.3f, "pd torque[0]");
    ExpectNear(torque(1), -10.0f + 1.8f - 0.5f, "pd torque[1]");
}

void JointCommandPacketRejectsWrongShape() {
    bool rejected = false;
    try {
        EncodeJointCommandPacket(types::MatXf::Zero(12, 4), 12);
    } catch (const std::exception&) {
        rejected = true;
    }
    Expect(rejected, "wrong-shape joint command matrix should be rejected");
}

types::MatXf SafeStandingCommand() {
    const int dof_num = 12;
    types::MatXf command = types::MatXf::Zero(dof_num, kJointCommandColumnCount);
    for (int i = 0; i < dof_num; ++i) {
        command(i, kJointCommandKp) = 20.0f;
        command(i, kJointCommandKd) = 1.0f;
        command(i, kJointCommandVelocity) = 0.0f;
        command(i, kJointCommandFeedForwardTorque) = 0.0f;
    }
    command.col(kJointCommandPosition) =
        types::Vec3f(0.0f, -0.8f, 1.6f).replicate(4, 1);
    return command;
}

void JointCommandLimitsAcceptSafeCommand() {
    const JointCommandLimitResult result =
        ValidateJointCommandLimits(SafeStandingCommand(), MiniCheetahJointCommandLimits(), 12);

    Expect(result.valid, "safe standing command should satisfy Mini Cheetah command limits");
    Expect(result.reason == JointCommandLimitReason::kValid, "safe command limit reason");
}

void JointCommandLimitsRejectEachUnsafeColumn() {
    const JointCommandLimits limits = MiniCheetahJointCommandLimits();

    types::MatXf command = SafeStandingCommand();
    command(0, kJointCommandPosition) = 2.0f;
    JointCommandLimitResult result = ValidateJointCommandLimits(command, limits, 12);
    Expect(!result.valid, "unsafe position should be rejected");
    Expect(result.reason == JointCommandLimitReason::kPositionLimit, "position limit reason");

    command = SafeStandingCommand();
    command(2, kJointCommandVelocity) = 25.0f;
    result = ValidateJointCommandLimits(command, limits, 12);
    Expect(!result.valid, "unsafe velocity should be rejected");
    Expect(result.reason == JointCommandLimitReason::kVelocityLimit, "velocity limit reason");

    command = SafeStandingCommand();
    command(0, kJointCommandKp) = 100.0f;
    result = ValidateJointCommandLimits(command, limits, 12);
    Expect(!result.valid, "unsafe kp should be rejected");
    Expect(result.reason == JointCommandLimitReason::kKpLimit, "kp limit reason");

    command = SafeStandingCommand();
    command(0, kJointCommandKp) = -1.0f;
    result = ValidateJointCommandLimits(command, limits, 12);
    Expect(!result.valid, "negative kp should be rejected");
    Expect(result.reason == JointCommandLimitReason::kKpLimit, "negative kp limit reason");

    command = SafeStandingCommand();
    command(0, kJointCommandKd) = 10.0f;
    result = ValidateJointCommandLimits(command, limits, 12);
    Expect(!result.valid, "unsafe kd should be rejected");
    Expect(result.reason == JointCommandLimitReason::kKdLimit, "kd limit reason");

    command = SafeStandingCommand();
    command(0, kJointCommandKd) = -1.0f;
    result = ValidateJointCommandLimits(command, limits, 12);
    Expect(!result.valid, "negative kd should be rejected");
    Expect(result.reason == JointCommandLimitReason::kKdLimit, "negative kd limit reason");

    command = SafeStandingCommand();
    command(2, kJointCommandFeedForwardTorque) = 30.0f;
    result = ValidateJointCommandLimits(command, limits, 12);
    Expect(!result.valid, "unsafe feedforward torque should be rejected");
    Expect(result.reason == JointCommandLimitReason::kFeedForwardTorqueLimit, "torque limit reason");

    command = SafeStandingCommand();
    command(0, kJointCommandPosition) = std::numeric_limits<float>::quiet_NaN();
    result = ValidateJointCommandLimits(command, limits, 12);
    Expect(!result.valid, "non-finite command should be rejected");
    Expect(result.reason == JointCommandLimitReason::kNonFinite, "non-finite limit reason");
}

void JointCommandClampMakesSlightlyUnsafeStandupSeedEncodable() {
    types::MatXf command = SafeStandingCommand();
    command(5, kJointCommandPosition) = 2.70621f;
    command(11, kJointCommandPosition) = 2.61663f;

    const JointCommandLimits limits = MiniCheetahJointCommandLimits();
    Expect(!ValidateJointCommandLimits(command, limits, 12).valid,
           "raw standup seed should exceed deployment position limits");

    const types::MatXf clamped = ClampJointCommandToLimits(command, limits, 12);
    Expect(ValidateJointCommandLimits(clamped, limits, 12).valid,
           "clamped standup seed should satisfy deployment limits");
    ExpectNear(clamped(5, kJointCommandPosition), 2.60f, "FL calf clamped position");
    ExpectNear(clamped(11, kJointCommandPosition), 2.60f, "RL calf clamped position");
    (void)EncodeJointCommandPacket(clamped, 12);
}

void JointDampingCommandHasSafeFallbackSemantics() {
    const types::VecXf kd = types::VecXf::Ones(12);
    const types::MatXf damping = BuildJointDampingCommand(kd, 12);

    Expect(IsDampingCommand(damping, 12), "damping command should be recognized as fallback");
    ExpectNear(damping.col(kJointCommandKp).norm(), 0.0f, "damping kp zero");
    ExpectNear(damping.col(kJointCommandPosition).norm(), 0.0f, "damping q_des zero");
    ExpectNear((damping.col(kJointCommandKd) - kd).norm(), 0.0f, "damping kd");
    ExpectNear(damping.col(kJointCommandVelocity).norm(), 0.0f, "damping dq_des zero");
    ExpectNear(damping.col(kJointCommandFeedForwardTorque).norm(), 0.0f, "damping tau_ff zero");
    Expect(ValidateJointCommandLimits(damping, MiniCheetahJointCommandLimits(), 12).valid,
           "damping command should satisfy limits");
}

void JointDampingCommandRejectsUnsafeFallbackInputs() {
    bool rejected = false;
    try {
        BuildJointDampingCommand(types::VecXf::Constant(12, -1.0f), 12);
    } catch (const std::exception&) {
        rejected = true;
    }
    Expect(rejected, "negative damping kd should be rejected");

    rejected = false;
    try {
        BuildJointDampingCommand(types::VecXf::Constant(12, 10.0f), 12);
    } catch (const std::exception&) {
        rejected = true;
    }
    Expect(rejected, "over-limit damping kd should be rejected");

    Expect(!IsDampingCommand(types::MatXf::Ones(6, kJointCommandColumnCount), 6),
           "non-canonical damping command should not be recognized");
}

}  // namespace

int main() {
    try {
        RobotStatePacketDecodesDeploymentInterfaceContract();
        CodecPublishesCanonicalJointOrder();
        RobotStatePacketRejectsNullPayload();
        RobotStatePacketRejectsOldLayoutWithoutBaseHeight();
        RobotStatePacketRejectsWrongSize();
        RobotStatePacketRejectsNonCanonicalDofCount();
        JointCommandPacketEncodesDeploymentInterfaceContractColumns();
        JointCommandPacketComputesPdCommandSemantics();
        JointCommandPacketRejectsWrongShape();
        JointCommandLimitsAcceptSafeCommand();
        JointCommandLimitsRejectEachUnsafeColumn();
        JointCommandClampMakesSlightlyUnsafeStandupSeedEncodable();
        JointDampingCommandHasSafeFallbackSemantics();
        JointDampingCommandRejectsUnsafeFallbackInputs();
    } catch (const std::exception& e) {
        std::cerr << "simulation_packet_codec_test failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
