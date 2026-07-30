#include "simulation_packet_codec.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace {

constexpr int kStateBaseHeightOffset = 0;
constexpr int kStateBaseLinearVelocityBodyOffset = 1;
constexpr int kStateRpyRadOffset = 4;
constexpr int kStateProperAccelerationBodyOffset = 7;
constexpr int kStateOmegaBodyOffset = 10;
constexpr int kStateJointDataOffset = 13;

int RobotStateFloatCount(int dof_num) { return kStateJointDataOffset + 3 * dof_num; }

void RequireCanonicalDofCount(int dof_num) {
    if (dof_num != static_cast<int>(CanonicalJointOrder().size())) {
        throw std::runtime_error("Mini Cheetah deployment interface requires canonical 12-DOF joint order");
    }
}

}  // namespace

const std::vector<std::string>& CanonicalJointOrder() {
    static const std::vector<std::string> joint_order = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    return joint_order;
}

JointCommandLimits MiniCheetahJointCommandLimits() {
    JointCommandLimits limits;
    limits.position_lower = types::Vec3f(-1.60f, -2.60f, -2.60f).replicate(4, 1);
    limits.position_upper = types::Vec3f(1.60f, 2.60f, 2.60f).replicate(4, 1);
    limits.velocity_abs_max = types::Vec3f(30.0f, 30.0f, 20.0f).replicate(4, 1);
    limits.kp_max = types::VecXf::Constant(12, 80.0f);
    limits.kd_max = types::VecXf::Constant(12, 5.0f);
    limits.feedforward_torque_abs_max = types::Vec3f(17.0f, 17.0f, 26.0f).replicate(4, 1);
    return limits;
}

std::size_t RobotStatePacketSize(int dof_num) {
    RequireCanonicalDofCount(dof_num);
    return sizeof(double) + sizeof(float) * RobotStateFloatCount(dof_num);
}

std::size_t JointCommandPacketSize(int dof_num) {
    RequireCanonicalDofCount(dof_num);
    return sizeof(float) * dof_num * kJointCommandColumnCount;
}

DecodedRobotStatePacket DecodeRobotStatePacket(
    const void* packet,
    std::size_t packet_size,
    int dof_num) {
    RequireCanonicalDofCount(dof_num);
    if (packet == nullptr) {
        throw std::runtime_error("Robot state packet payload must not be null");
    }
    const std::size_t expected_size = RobotStatePacketSize(dof_num);
    if (packet_size != expected_size) {
        throw std::runtime_error("Robot state packet size does not match deployment interface contract");
    }

    DecodedRobotStatePacket decoded;
    decoded.joint_pos = types::VecXf::Zero(dof_num);
    decoded.joint_vel = types::VecXf::Zero(dof_num);
    decoded.joint_tau = types::VecXf::Zero(dof_num);

    std::memcpy(&decoded.timestamp, packet, sizeof(double));

    std::vector<float> data(RobotStateFloatCount(dof_num));
    std::memcpy(data.data(), static_cast<const char*>(packet) + sizeof(double), sizeof(float) * data.size());

    decoded.base_height = data[kStateBaseHeightOffset];
    decoded.base_lin_vel_body = Eigen::Map<types::Vec3f>(data.data() + kStateBaseLinearVelocityBodyOffset, 3);
    decoded.rpy_rad = Eigen::Map<types::Vec3f>(data.data() + kStateRpyRadOffset, 3);
    decoded.proper_acc_body = Eigen::Map<types::Vec3f>(data.data() + kStateProperAccelerationBodyOffset, 3);
    decoded.omega_body = Eigen::Map<types::Vec3f>(data.data() + kStateOmegaBodyOffset, 3);
    decoded.joint_pos = Eigen::Map<types::VecXf>(data.data() + kStateJointDataOffset, dof_num);
    decoded.joint_vel = Eigen::Map<types::VecXf>(data.data() + kStateJointDataOffset + dof_num, dof_num);
    decoded.joint_tau = Eigen::Map<types::VecXf>(data.data() + kStateJointDataOffset + 2 * dof_num, dof_num);

    return decoded;
}

std::vector<char> EncodeJointCommandPacket(
    const types::MatXf& command,
    int dof_num) {
    RequireCanonicalDofCount(dof_num);
    const JointCommandLimitResult limit_result =
        ValidateJointCommandLimits(command, MiniCheetahJointCommandLimits(), dof_num);
    if (!limit_result.valid) {
        throw std::runtime_error("Joint command violates Mini Cheetah deployment interface limits");
    }

    std::vector<char> packet(JointCommandPacketSize(dof_num));
    std::memcpy(packet.data(), command.data(), packet.size());
    return packet;
}

types::VecXf ComputePdCommandTorque(
    const types::MatXf& command,
    const types::VecXf& joint_pos,
    const types::VecXf& joint_vel,
    int dof_num) {
    if (command.rows() != dof_num || command.cols() != kJointCommandColumnCount ||
        joint_pos.size() != dof_num || joint_vel.size() != dof_num) {
        throw std::runtime_error("PD command inputs do not match deployment interface contract");
    }

    return command.col(kJointCommandKp).cwiseProduct(command.col(kJointCommandPosition) - joint_pos) +
           command.col(kJointCommandKd).cwiseProduct(command.col(kJointCommandVelocity) - joint_vel) +
           command.col(kJointCommandFeedForwardTorque);
}

namespace {

JointCommandLimitResult InvalidResult(JointCommandLimitReason reason, int joint_index, int column) {
    JointCommandLimitResult result;
    result.valid = false;
    result.reason = reason;
    result.joint_index = joint_index;
    result.column = column;
    return result;
}

bool IsFinite(float value) {
    return std::isfinite(value);
}

bool LimitsHaveCanonicalSize(const JointCommandLimits& limits, int dof_num) {
    return limits.position_lower.size() == dof_num &&
           limits.position_upper.size() == dof_num &&
           limits.velocity_abs_max.size() == dof_num &&
           limits.kp_max.size() == dof_num &&
           limits.kd_max.size() == dof_num &&
           limits.feedforward_torque_abs_max.size() == dof_num;
}

}  // namespace

JointCommandLimitResult ValidateJointCommandLimits(
    const types::MatXf& command,
    const JointCommandLimits& limits,
    int dof_num) {
    RequireCanonicalDofCount(dof_num);
    if (command.rows() != dof_num ||
        command.cols() != kJointCommandColumnCount ||
        !LimitsHaveCanonicalSize(limits, dof_num)) {
        return InvalidResult(JointCommandLimitReason::kShape, -1, -1);
    }

    for (int joint = 0; joint < dof_num; ++joint) {
        for (int column = 0; column < kJointCommandColumnCount; ++column) {
            if (!IsFinite(command(joint, column))) {
                return InvalidResult(JointCommandLimitReason::kNonFinite, joint, column);
            }
        }
        if (command(joint, kJointCommandPosition) < limits.position_lower(joint) ||
            command(joint, kJointCommandPosition) > limits.position_upper(joint)) {
            return InvalidResult(JointCommandLimitReason::kPositionLimit, joint, kJointCommandPosition);
        }
        if (std::fabs(command(joint, kJointCommandVelocity)) > limits.velocity_abs_max(joint)) {
            return InvalidResult(JointCommandLimitReason::kVelocityLimit, joint, kJointCommandVelocity);
        }
        if (command(joint, kJointCommandKp) < 0.0f ||
            command(joint, kJointCommandKp) > limits.kp_max(joint)) {
            return InvalidResult(JointCommandLimitReason::kKpLimit, joint, kJointCommandKp);
        }
        if (command(joint, kJointCommandKd) < 0.0f ||
            command(joint, kJointCommandKd) > limits.kd_max(joint)) {
            return InvalidResult(JointCommandLimitReason::kKdLimit, joint, kJointCommandKd);
        }
        if (std::fabs(command(joint, kJointCommandFeedForwardTorque)) >
            limits.feedforward_torque_abs_max(joint)) {
            return InvalidResult(
                JointCommandLimitReason::kFeedForwardTorqueLimit,
                joint,
                kJointCommandFeedForwardTorque);
        }
    }

    JointCommandLimitResult result;
    result.valid = true;
    result.reason = JointCommandLimitReason::kValid;
    return result;
}

types::MatXf ClampJointCommandToLimits(
    const types::MatXf& command,
    const JointCommandLimits& limits,
    int dof_num) {
    RequireCanonicalDofCount(dof_num);
    if (command.rows() != dof_num ||
        command.cols() != kJointCommandColumnCount ||
        !LimitsHaveCanonicalSize(limits, dof_num)) {
        throw std::runtime_error("Joint command clamp inputs do not match deployment interface contract");
    }

    types::MatXf clamped = command;
    for (int joint = 0; joint < dof_num; ++joint) {
        for (int column = 0; column < kJointCommandColumnCount; ++column) {
            if (!IsFinite(command(joint, column))) {
                throw std::runtime_error("Joint command clamp rejects non-finite values");
            }
        }

        clamped(joint, kJointCommandPosition) =
            std::clamp(command(joint, kJointCommandPosition),
                       limits.position_lower(joint),
                       limits.position_upper(joint));
        clamped(joint, kJointCommandVelocity) =
            std::clamp(command(joint, kJointCommandVelocity),
                       -limits.velocity_abs_max(joint),
                       limits.velocity_abs_max(joint));
        clamped(joint, kJointCommandKp) =
            std::clamp(command(joint, kJointCommandKp), 0.0f, limits.kp_max(joint));
        clamped(joint, kJointCommandKd) =
            std::clamp(command(joint, kJointCommandKd), 0.0f, limits.kd_max(joint));
        clamped(joint, kJointCommandFeedForwardTorque) =
            std::clamp(command(joint, kJointCommandFeedForwardTorque),
                       -limits.feedforward_torque_abs_max(joint),
                       limits.feedforward_torque_abs_max(joint));
    }
    return clamped;
}

types::MatXf BuildJointDampingCommand(
    const types::VecXf& kd,
    int dof_num) {
    RequireCanonicalDofCount(dof_num);
    if (kd.size() != dof_num) {
        throw std::runtime_error("Damping command kd must match canonical 12-DOF joint order");
    }

    types::MatXf command = types::MatXf::Zero(dof_num, kJointCommandColumnCount);
    command.col(kJointCommandKd) = kd;
    const JointCommandLimitResult limit_result =
        ValidateJointCommandLimits(command, MiniCheetahJointCommandLimits(), dof_num);
    if (!limit_result.valid) {
        throw std::runtime_error("Damping command violates Mini Cheetah deployment interface limits");
    }
    return command;
}

bool IsDampingCommand(
    const types::MatXf& command,
    int dof_num) {
    if (dof_num != static_cast<int>(CanonicalJointOrder().size())) {
        return false;
    }
    if (command.rows() != dof_num || command.cols() != kJointCommandColumnCount) {
        return false;
    }
    return command.col(kJointCommandKp).isZero(1e-6f) &&
           command.col(kJointCommandPosition).isZero(1e-6f) &&
           (command.col(kJointCommandKd).array() > 0.0f).all() &&
           command.col(kJointCommandVelocity).isZero(1e-6f) &&
           command.col(kJointCommandFeedForwardTorque).isZero(1e-6f);
}
