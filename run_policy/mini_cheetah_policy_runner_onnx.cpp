/**
 * @file mini_cheetah_policy_runner_onnx.cpp
 * @brief Implementation of MiniCheetahPolicyRunnerONNX.
 *        This is the ONLY file that includes onnxruntime headers, so only
 *        this translation unit needs to be recompiled when policy logic changes.
 * @author Bo (Percy) Peng
 * @version 1.0
 * @date 2025-08-10
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 */

#include "mini_cheetah_policy_runner_onnx.h"

#include <onnxruntime_cxx_api.h>

#include "basic_function.hpp"
#include "policy_metadata.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <unordered_map>

namespace {
std::string ResolvePolicyPath() {
    const std::filesystem::path cwd = GetAbsPath();
    const std::filesystem::path from_project_root = cwd / "policy/ppo/policy.onnx";
    const std::filesystem::path from_build_dir = cwd / "../policy/ppo/policy.onnx";

    if (std::filesystem::exists(from_project_root)) {
        return from_project_root.lexically_normal().string();
    }
    return from_build_dir.lexically_normal().string();
}
}

// ---------------------------------------------------------------------------
// OrtImpl — all onnxruntime objects live here, invisible to callers
// ---------------------------------------------------------------------------
struct MiniCheetahPolicyRunnerONNX::OrtImpl {
    Ort::Env env;
    Ort::SessionOptions session_options;
    Ort::MemoryInfo memory_info;
    Ort::Session session;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    OrtImpl()
        : env(ORT_LOGGING_LEVEL_WARNING, "ONNXPolicy"),
          session_options(),
          memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
          session(nullptr) {}
};

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
MiniCheetahPolicyRunnerONNX::MiniCheetahPolicyRunnerONNX(std::string policy_name)
    : PolicyRunnerBase(policy_name),
      ort_(std::make_unique<OrtImpl>()),
      gravity_direction(0., 0., -1.),
      joint_pos_rl(12),
      joint_vel_rl(12) {

    model_path_ = ResolvePolicyPath();
    metadata_path_ = ResolvePolicyMetadataPath();
    const PolicyMetadata metadata = LoadPolicyMetadata(metadata_path_);
    obs_dim_ = metadata.obs_dim;
    act_dim_ = metadata.action_dim;
    if (act_dim_ != 12) {
        throw std::runtime_error("MiniCheetahPolicyRunnerONNX requires a 12-dim action policy");
    }

    std::cout << "[ONNX INIT] Loading model: " << model_path_ << std::endl;
    std::cout << "[ONNX INIT] Loading metadata: " << metadata_path_ << std::endl;

    ort_->session_options.SetIntraOpNumThreads(1);
    ort_->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    ort_->session = Ort::Session(ort_->env, model_path_.c_str(), ort_->session_options);
    std::cout << "[ONNX INIT] Model loaded successfully.\n";

    ort_->input_names  = {"obs"};
    ort_->output_names = {"actions"};

    robot_order = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

    policy_order = metadata.joint_order;

    dof_pos_default_policy = Eigen::Map<const Eigen::VectorXf>(
        metadata.default_joint_pos.data(), metadata.default_joint_pos.size());

    kp_ = 20. * VecXf::Ones(12);
    kd_ =  1. * VecXf::Ones(12);
    max_cmd_vel_ << 0.8, 0.8, 0.8;
    omega_scale_ = metadata.omega_scale;
    lin_vel_scale_ = metadata.lin_vel_scale;
    dof_vel_scale_ = metadata.dof_vel_scale;
    decimation_ = metadata.decimation;

    current_obs_ = VecXf::Zero(obs_dim_);
    tmp_action = VecXf::Zero(act_dim_);
    action = VecXf::Zero(act_dim_);
    last_action = VecXf::Zero(act_dim_);
    transition_start_joint_pos_ = VecXf::Zero(act_dim_);
    ra.goal_joint_pos = VecXf::Zero(act_dim_);
    ra.goal_joint_vel = VecXf::Zero(act_dim_);
    ra.tau_ff         = VecXf::Zero(act_dim_);
    ra.kp = kp_;
    ra.kd = kd_;

    robot2policy_idx = generate_permutation(robot_order, policy_order);
    policy2robot_idx = generate_permutation(policy_order, robot_order);
    action_scale_robot.resize(act_dim_);
    dof_pos_default_robot.setZero(act_dim_);
    for (int i = 0; i < act_dim_; ++i) {
        action_scale_robot[i] = metadata.action_scale[policy2robot_idx[i]];
        dof_pos_default_robot(i) = metadata.default_joint_pos[policy2robot_idx[i]];
        std::cout << "robot2policy_idx[" << i << "]: " << robot2policy_idx[i] << std::endl;
        std::cout << "policy2robot_idx[" << i << "]: " << policy2robot_idx[i] << std::endl;
    }

    // Warm-up / sanity check
    for (int i = 0; i < 2; ++i) {
        VecXf dummy_input = VecXf::Ones(obs_dim_);
        std::array<int64_t, 2> input_shape{1, obs_dim_};

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            ort_->memory_info, dummy_input.data(), obs_dim_,
            input_shape.data(), input_shape.size());

        auto reaction = ort_->session.Run(
            Ort::RunOptions{nullptr},
            ort_->input_names.data(), &input_tensor, 1,
            ort_->output_names.data(), 1);

        std::cout << policy_name_ << " ONNX policy network test success" << std::endl;
    }
}

// The destructor must be defined in the .cpp where OrtImpl is complete.
MiniCheetahPolicyRunnerONNX::~MiniCheetahPolicyRunnerONNX() = default;

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------
void MiniCheetahPolicyRunnerONNX::DisplayPolicyInfo() {
    std::cout << "ONNX policy: " << policy_name_ << "\n";
    std::cout << "path: " << model_path_ << "\n";
    std::cout << "metadata: " << metadata_path_ << "\n";
    std::cout << "obs_dim: " << obs_dim_ << ", action_dim: " << act_dim_ << "\n";
}

void MiniCheetahPolicyRunnerONNX::OnEnter() {
    run_cnt_ = 0;
    current_obs_.setZero(obs_dim_);
    last_action.setZero(act_dim_);
    transition_start_recorded_ = false;
    std::cout << "[ONNX ENTER] PolicyRunner entered: " << policy_name_ << std::endl;
}

RobotAction MiniCheetahPolicyRunnerONNX::GetRobotAction(const RobotBasicState& ro) {
    Vec3f base_lin_vel = ro.base_lin_vel * lin_vel_scale_;
    Vec3f base_omega       = ro.base_omega * omega_scale_;
    Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
    Vec3f cmd_vel          = ro.cmd_vel_normlized.cwiseProduct(max_cmd_vel_);

    for (int i = 0; i < act_dim_; ++i) {
        joint_pos_rl(i) = ro.joint_pos(robot2policy_idx[i]);
        joint_vel_rl(i) = ro.joint_vel(robot2policy_idx[i]) * dof_vel_scale_;
    }
    joint_pos_rl -= dof_pos_default_policy;

    current_obs_.setZero(obs_dim_);
    current_obs_ << base_lin_vel,
                    base_omega,
                    projected_gravity,
                    cmd_vel,
                    joint_pos_rl,
                    joint_vel_rl,
                    last_action;

    std::array<int64_t, 2> input_shape{1, obs_dim_};

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        ort_->memory_info,
        current_obs_.data(), current_obs_.size(),
        input_shape.data(), input_shape.size());

    std::vector<Ort::Value> inputs;
    inputs.emplace_back(std::move(input_tensor));

    auto output_tensors = ort_->session.Run(
        Ort::RunOptions{nullptr},
        ort_->input_names.data(), inputs.data(), 1,
        ort_->output_names.data(), 1);

    float* action_data = output_tensors[0].GetTensorMutableData<float>();
    Eigen::Map<Eigen::MatrixXf> act(action_data, act_dim_, 1);
    action      = VecXf(act);
    VecXf clipped_action = action.cwiseMax(-action_clip_).cwiseMin(action_clip_);
    last_action = clipped_action;

    for (int i = 0; i < act_dim_; ++i) {
        tmp_action(i)  = clipped_action(policy2robot_idx[i]);
        tmp_action(i) *= action_scale_robot[i];
    }
    tmp_action += dof_pos_default_robot;

    for (int leg = 0; leg < 4; ++leg) {
        const int abad = 3 * leg;
        const int thigh = abad + 1;
        const int calf = abad + 2;
        tmp_action(abad) = std::max(-1.0f, std::min(1.0f, tmp_action(abad)));
        tmp_action(thigh) = std::max(-1.8f, std::min(0.8f, tmp_action(thigh)));
        tmp_action(calf) = std::max(0.4f, std::min(2.6f, tmp_action(calf)));
    }

    if (!transition_start_recorded_) {
        transition_start_joint_pos_ = ro.joint_pos;
        transition_start_recorded_ = true;
    }
    const float blend = std::min(1.0f, static_cast<float>(run_cnt_) / static_cast<float>(transition_steps_));
    VecXf blended_target = (1.0f - blend) * transition_start_joint_pos_ + blend * tmp_action;

    if (run_cnt_ < 5 || run_cnt_ % 50 == 0) {
        std::cout << "[RL DEBUG] run_cnt: " << run_cnt_ << std::endl;
        std::cout << "[RL DEBUG] transition_blend: " << blend << std::endl;
        std::cout << "[RL DEBUG] base_lin_vel(obs scaled): " << base_lin_vel.transpose() << std::endl;
        std::cout << "[RL DEBUG] base_omega(obs scaled): " << base_omega.transpose() << std::endl;
        std::cout << "[RL DEBUG] projected_gravity: " << projected_gravity.transpose() << std::endl;
        std::cout << "[RL DEBUG] cmd_vel: " << cmd_vel.transpose() << std::endl;
        std::cout << "[RL DEBUG] joint_pos_minus_default: " << joint_pos_rl.transpose() << std::endl;
        std::cout << "[RL DEBUG] raw_action: " << action.transpose() << std::endl;
        std::cout << "[RL DEBUG] clipped_action: " << clipped_action.transpose() << std::endl;
        std::cout << "[RL DEBUG] target_joint_pos: " << tmp_action.transpose() << std::endl;
        std::cout << "[RL DEBUG] blended_target_joint_pos: " << blended_target.transpose() << std::endl;
    }

    ra.goal_joint_pos = blended_target;
    ra.goal_joint_vel = VecXf::Zero(act_dim_);
    ra.tau_ff         = VecXf::Zero(act_dim_);
    ra.kp = kp_;
    ra.kd = kd_;
    ++run_cnt_;

    return ra;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------
std::vector<int> MiniCheetahPolicyRunnerONNX::generate_permutation(
    const std::vector<std::string>& from,
    const std::vector<std::string>& to,
    int default_index)
{
    std::unordered_map<std::string, int> idx_map;
    for (int i = 0; i < (int)from.size(); ++i)
        idx_map[from[i]] = i;

    std::vector<int> perm;
    for (const auto& name : to) {
        auto it = idx_map.find(name);
        perm.push_back(it != idx_map.end() ? it->second : default_index);
    }
    return perm;
}
