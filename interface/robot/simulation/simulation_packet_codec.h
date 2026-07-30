#pragma once

#include "common_types.h"

#include <cstddef>
#include <string>
#include <vector>

constexpr int kJointCommandKp = 0;
constexpr int kJointCommandPosition = 1;
constexpr int kJointCommandKd = 2;
constexpr int kJointCommandVelocity = 3;
constexpr int kJointCommandFeedForwardTorque = 4;
constexpr int kJointCommandColumnCount = 5;

struct DecodedRobotStatePacket {
    double timestamp = 0.0;
    float base_height = 0.0f;
    types::Vec3f base_lin_vel_body = types::Vec3f::Zero();
    types::Vec3f rpy_rad = types::Vec3f::Zero();
    types::Vec3f proper_acc_body = types::Vec3f::Zero();
    types::Vec3f omega_body = types::Vec3f::Zero();
    types::VecXf joint_pos;
    types::VecXf joint_vel;
    types::VecXf joint_tau;
};

enum class JointCommandLimitReason {
    kValid,
    kShape,
    kPositionLimit,
    kVelocityLimit,
    kKpLimit,
    kKdLimit,
    kFeedForwardTorqueLimit,
    kNonFinite,
};

struct JointCommandLimitResult {
    bool valid = false;
    JointCommandLimitReason reason = JointCommandLimitReason::kValid;
    int joint_index = -1;
    int column = -1;
};

struct JointCommandLimits {
    types::VecXf position_lower;
    types::VecXf position_upper;
    types::VecXf velocity_abs_max;
    types::VecXf kp_max;
    types::VecXf kd_max;
    types::VecXf feedforward_torque_abs_max;
};

const std::vector<std::string>& CanonicalJointOrder();
JointCommandLimits MiniCheetahJointCommandLimits();

std::size_t RobotStatePacketSize(int dof_num);
std::size_t JointCommandPacketSize(int dof_num);

DecodedRobotStatePacket DecodeRobotStatePacket(
    const void* packet,
    std::size_t packet_size,
    int dof_num);

std::vector<char> EncodeJointCommandPacket(
    const types::MatXf& command,
    int dof_num);

types::VecXf ComputePdCommandTorque(
    const types::MatXf& command,
    const types::VecXf& joint_pos,
    const types::VecXf& joint_vel,
    int dof_num);

JointCommandLimitResult ValidateJointCommandLimits(
    const types::MatXf& command,
    const JointCommandLimits& limits,
    int dof_num);

types::MatXf BuildJointDampingCommand(
    const types::VecXf& kd,
    int dof_num);

bool IsDampingCommand(
    const types::MatXf& command,
    int dof_num);
