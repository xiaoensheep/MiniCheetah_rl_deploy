/**
 * @file rl_control_state_onnx.hpp
 * @brief rl policy runnning state using onnx
 * @author Bo (Percy) Peng
 * @version 1.0
 * @date 2025-08-10
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */




#pragma once

#include "state_base.h"
#include "policy_runner_base.hpp"
#include "mini_cheetah_policy_runner_onnx.h"
#include "episode_log_builder.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>


class RLControlStateONNX : public StateBase
{
private:
    RobotBasicState rbs_;
    int state_run_cnt_;

    std::shared_ptr<PolicyRunnerBase> policy_ptr_;
    std::shared_ptr<MiniCheetahPolicyRunnerONNX> onnx_policy_runner_;


    
    std::thread run_policy_thread_;
    bool start_flag_ = true;

    float policy_cost_time_ = 1;
    double previous_policy_time_ = 0.0;

#ifdef BUILD_SIMULATION
    std::unique_ptr<EpisodeLogger> episode_logger_;
    std::string episode_log_path_;

    std::string ResolveEpisodeLogPath() const {
        const char* env_path = std::getenv("MINI_CHEETAH_EPISODE_LOG");
        if (env_path != nullptr) {
            const std::string requested_path(env_path);
            if (!requested_path.empty()) {
                const std::filesystem::path path(requested_path);
                const std::filesystem::path parent = path.parent_path();
                if (!parent.empty()) {
                    std::filesystem::create_directories(parent);
                }
                return path.lexically_normal().string();
            }
        }

        const std::filesystem::path log_dir = std::filesystem::path(GetAbsPath()) / "logs";
        std::filesystem::create_directories(log_dir);
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        return (log_dir / ("sim_episode_" + std::to_string(now_ms) + ".jsonl")).lexically_normal().string();
    }

    static bool ActionClipApplied(const ReplayPolicyOutput& output) {
        return output.raw_action.size() == output.clipped_action.size() &&
               output.raw_action.size() > 0 &&
               (output.raw_action - output.clipped_action).cwiseAbs().maxCoeff() > 1e-6f;
    }

    bool RuntimeSafetyGatePassed(const RobotBasicState& robot_state) const {
        return uc_ptr_->GetUserCommand().target_mode != int(RobotMotionState::JointDamping) &&
               std::fabs(robot_state.base_rpy(0)) <= 30.0f / 180.0f * M_PI &&
               std::fabs(robot_state.base_rpy(1)) <= 45.0f / 180.0f * M_PI;
    }

    std::string RuntimeSafetyGateReason(const RobotBasicState& robot_state) const {
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) {
            return "user_requested_damping";
        }
        if (std::fabs(robot_state.base_rpy(0)) > 30.0f / 180.0f * M_PI ||
            std::fabs(robot_state.base_rpy(1)) > 45.0f / 180.0f * M_PI) {
            return "posture_unsafe";
        }
        return "running_safe";
    }

    void AppendEpisodeLog(const RobotBasicState& robot_state,
                          const RobotAction& robot_action,
                          double timestamp,
                          float policy_control_dt) {
        if (!episode_logger_) {
            return;
        }

        const ReplayPolicyOutput& policy_output = onnx_policy_runner_->GetLastReplayOutput();
        EpisodeLogContext context;
        context.timestamp = timestamp;
        context.state_machine_state = state_name_;
        context.user_command = uc_ptr_->GetUserCommand();
        context.policy_output = policy_output;
        context.commanded_target_joint_pos = robot_action.goal_joint_pos;
        context.policy_entry_gate_passed = RuntimeSafetyGatePassed(robot_state);
        context.policy_entry_gate_reason = RuntimeSafetyGateReason(robot_state);
        context.clamp_applied = ActionClipApplied(policy_output);
        context.clamp_reason = context.clamp_applied ? "action_clip" : "none";
        context.policy_inference_ms = policy_cost_time_;
        context.control_dt = policy_control_dt;

        episode_logger_->Append(BuildEpisodeLogRecord(
            robot_state, onnx_policy_runner_->GetLastObservation(), context));
    }
#endif

    void UpdateRobotObservation(){
        rbs_.base_rpy     = ri_ptr_->GetImuRpy();
        rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
        rbs_.projected_gravity = RmToProjectedGravity(rbs_.base_rot_mat);
        rbs_.base_lin_vel = ri_ptr_->GetBaseLinearVelocity();
        rbs_.base_omega   = ri_ptr_->GetImuOmega();
        rbs_.base_acc     = ri_ptr_->GetImuAcc();
        rbs_.joint_pos    = ri_ptr_->GetJointPosition();
        rbs_.joint_vel    = ri_ptr_->GetJointVelocity();
        rbs_.joint_tau    = ri_ptr_->GetJointTorque();
        // static Vec3f cmd_vel;
        // Vec3f cmd_vel_input = Vec3f(uc_ptr_->GetUserCommand().forward_vel_scale, 
        //                             uc_ptr_->GetUserCommand().side_vel_scale, 
        //                             uc_ptr_->GetUserCommand().turnning_vel_scale);

        // Eigen::Vector3f vel_delta = cmd_vel_input - cmd_vel;
        // const Eigen::Vector3f vel_delta_const(0.0015, 0.001, 0.0012);
        // for(int i=0;i<3;++i){
        //     if(fabs(vel_delta(i)) > vel_delta_const(i)) vel_delta(i) = Sign(vel_delta(i))*vel_delta_const(i);
        // }
        // cmd_vel+=vel_delta;           
        // rbs_.cmd_vel_normlized = cmd_vel;
        rbs_.cmd_vel_normlized = Vec3f(uc_ptr_->GetUserCommand().forward_vel_scale, 
                                    uc_ptr_->GetUserCommand().side_vel_scale, 
                                    uc_ptr_->GetUserCommand().turnning_vel_scale);
        
    }

    void PolicyRunner(){
        int run_cnt_record = -1;
        while (start_flag_){
            
            if(state_run_cnt_%policy_ptr_->decimation_ == 0 && state_run_cnt_ != run_cnt_record){
                RobotBasicState policy_robot_state = rbs_;
                timespec start_timestamp, end_timestamp;
                clock_gettime(CLOCK_MONOTONIC,&start_timestamp);
                auto ra = policy_ptr_->GetRobotAction(policy_robot_state);
                MatXf res = ra.ConvertToMat();
                ri_ptr_->SetJointCommand(res); // (current torque, not last torque, video content slip of the tongue)
                run_cnt_record = state_run_cnt_;
                clock_gettime(CLOCK_MONOTONIC,&end_timestamp);
                policy_cost_time_ = (end_timestamp.tv_sec-start_timestamp.tv_sec)*1e3 
                                    +(end_timestamp.tv_nsec-start_timestamp.tv_nsec)/1e6;
#ifdef BUILD_SIMULATION
                const double policy_timestamp = ri_ptr_->GetInterfaceTimeStamp();
                const float policy_control_dt = previous_policy_time_ > 0.0
                    ? static_cast<float>(policy_timestamp - previous_policy_time_)
                    : 0.0f;
                previous_policy_time_ = policy_timestamp;
                AppendEpisodeLog(policy_robot_state, ra, policy_timestamp, policy_control_dt);
#endif
                // std::cout << "cost_time:  " << policy_cost_time_ << " ms\n";
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

public:
    RLControlStateONNX(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
        std::memset(&rbs_, 0, sizeof(rbs_));
        onnx_policy_runner_ = std::make_shared<MiniCheetahPolicyRunnerONNX>("test_onnx");
        policy_ptr_ = onnx_policy_runner_;
        if(!policy_ptr_){
            std::cerr << "[ERROR] Failed to initialize ONNX policy runner." << std::endl;
            exit(0);
        }  
        policy_ptr_->DisplayPolicyInfo();
        }
    ~RLControlStateONNX(){}

    virtual void OnEnter() {
        state_run_cnt_ = -1;
        start_flag_ = true;
        previous_policy_time_ = 0.0;
#ifdef BUILD_SIMULATION
        try {
            episode_log_path_ = ResolveEpisodeLogPath();
            if (!episode_log_path_.empty()) {
                episode_logger_ = std::make_unique<EpisodeLogger>(episode_log_path_);
                std::cout << "[EPISODE LOG] writing simulation records to: "
                          << episode_log_path_ << std::endl;
            }
        } catch (const std::exception& e) {
            episode_logger_.reset();
            std::cerr << "[EPISODE LOG] disabled: " << e.what() << std::endl;
        }
#endif
        UpdateRobotObservation();
        std::cout << "[RL ENTER] rpy(deg): " << (180.0 / M_PI * rbs_.base_rpy).transpose() << std::endl;
        std::cout << "[RL ENTER] joint_pos: " << rbs_.joint_pos.transpose() << std::endl;
        policy_ptr_->OnEnter();
        run_policy_thread_ = std::thread(std::bind(&RLControlStateONNX::PolicyRunner, this));
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    };

    virtual void OnExit() { 
        start_flag_ = false;
        run_policy_thread_.join();
        state_run_cnt_ = -1;
#ifdef BUILD_SIMULATION
        episode_logger_.reset();
#endif
    }

    virtual void Run() {
        UpdateRobotObservation();
        ds_ptr_->InsertScopeData(0, policy_cost_time_);
        state_run_cnt_++;
    }

    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        return PostureUnsafeCheck();
    }

    bool PostureUnsafeCheck(){
        Vec3f rpy = ri_ptr_->GetImuRpy();
        if(fabs(rpy(0)) > 30./180*M_PI || fabs(rpy(1)) > 45./180*M_PI){
            std::cout << "posture value: " << 180./M_PI*rpy.transpose() << std::endl;
            return true;
        }
        return false;
    }

    virtual StateName GetNextStateName() {
        return StateName::kRLControl;
    }
};
