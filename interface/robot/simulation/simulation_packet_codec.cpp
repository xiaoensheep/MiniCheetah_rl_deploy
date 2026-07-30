#include "simulation_packet_codec.h"

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
    if (command.rows() != dof_num || command.cols() != kJointCommandColumnCount) {
        throw std::runtime_error("Joint command matrix shape must be dof_num x 5");
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
