/**
 * @file standup_state.hpp
 * @brief from sit state to stand state
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#pragma once

#include "state_base.h"
#include "policy_entry_gate.h"
#include "policy_metadata.h"
#include "simulation_packet_codec.h"

class StandUpState : public StateBase{
private:
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    float time_stamp_record_, run_time_, previous_run_time_, control_dt_;
    VecXf goal_joint_pos_, kp_, kd_;
    MatXf joint_cmd_;
    PolicyEntryGateConfig policy_entry_gate_config_;
    float stand_duration_ = 2.;
    PolicyEntryGateReason last_policy_entry_gate_reason_ = PolicyEntryGateReason::kReady;
    float last_policy_entry_gate_print_time_ = -10000.0f;

    void GetRobotJointValue(){
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
        previous_run_time_ = run_time_;
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        if(previous_run_time_ > 0.0f && run_time_ > previous_run_time_){
            control_dt_ = run_time_ - previous_run_time_;
        }
    }

    void RecordJointData(){
        init_joint_pos_ = current_joint_pos_;
        init_joint_vel_ = current_joint_vel_;
        time_stamp_record_ = run_time_;
    }

    float GetCubicSplinePos(float x0, float v0, float xf, float vf, float t, float T){
        if(t >= T) return xf;
        float a, b, c, d;
        d = x0;
        c = v0;
        a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
        b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
        return a*pow(t, 3)+b*pow(t, 2)+c*t+d;
    }
    float GetCubicSplineVel(float x0, float v0, float xf, float vf, float t, float T){
        if(t >= T) return 0;
        float a, b, c;
        c = v0;
        a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
        b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
        return 3.*a*pow(t, 2) + 2.*b*t + c;
    }

    float GetHipYPosByHeight(float h){
        float l1 = cp_ptr_->thigh_len_;
        float l2 = cp_ptr_->shank_len_;
        float default_pos = (cp_ptr_->fl_joint_lower_(1)+cp_ptr_->fl_joint_upper_(1)) / 2.;
        if(fabs(h) >= l1 + l2) {
            std::cerr << "error height input" << std::endl;
            return 0;
        }
        float theta = -acos((l1*l1+h*h-l2*l2)/(2.*h*l1));
        theta = LimitNumber(theta, cp_ptr_->fl_joint_lower_(1), cp_ptr_->fl_joint_upper_(1));
        return theta;
    }

    float GetKneePosByHeight(float h){
        float l1 = cp_ptr_->thigh_len_;
        float l2 = cp_ptr_->shank_len_;
        float default_pos = (cp_ptr_->fl_joint_lower_(2)+cp_ptr_->fl_joint_upper_(2)) / 2.;
        if(fabs(h) >= l1 + l2) {
            std::cerr << "error height input" << std::endl;
            return 0;
        }
        float theta = M_PI-acos((l1*l1+l2*l2-h*h)/(2*l1*l2));
        theta = LimitNumber(theta, cp_ptr_->fl_joint_lower_(2), cp_ptr_->fl_joint_upper_(2));
        return theta;
    }

    void MaybePrintPolicyEntryGateRefusal(const PolicyEntryGateResult& gate_result) {
        const bool reason_changed = gate_result.reason != last_policy_entry_gate_reason_;
        const bool print_interval_elapsed = run_time_ - last_policy_entry_gate_print_time_ > 0.5f;
        if (!reason_changed && !print_interval_elapsed) {
            return;
        }

        float max_joint_error = 0.0f;
        if (current_joint_pos_.size() == policy_entry_gate_config_.default_joint_pos.size()) {
            max_joint_error = (current_joint_pos_ - policy_entry_gate_config_.default_joint_pos)
                                  .cwiseAbs()
                                  .maxCoeff();
        }

        std::cout << "[POLICY ENTRY GATE] refused RLControl: "
                  << PolicyEntryGateReasonToString(gate_result.reason)
                  << " | max_joint_error: " << max_joint_error
                  << " | base_height: " << ri_ptr_->GetBaseHeight()
                  << std::endl;
        last_policy_entry_gate_reason_ = gate_result.reason;
        last_policy_entry_gate_print_time_ = run_time_;
    }

public:
    StandUpState(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
            const PolicyMetadata policy_metadata = LoadPolicyMetadata(ResolvePolicyMetadataPath());
            goal_joint_pos_ = Eigen::Map<const Eigen::VectorXf>(
                policy_metadata.default_joint_pos.data(), policy_metadata.default_joint_pos.size());
            policy_entry_gate_config_.default_joint_pos = goal_joint_pos_;
            policy_entry_gate_config_.target_base_height = policy_metadata.target_base_height;
            policy_entry_gate_config_.max_control_dt = 1.5f / policy_metadata.pd_update_frequency_hz;
#ifdef BUILD_SIMULATION
            policy_entry_gate_config_.max_joint_pos_error = 0.35f;
            policy_entry_gate_config_.max_base_height_error = 0.08f;
#endif
            kp_ = VecXf(12);
            kd_ = VecXf(12);     
            kp_ = cp_ptr_->swing_leg_kp_.replicate(4, 1);
            kd_ = cp_ptr_->swing_leg_kd_.replicate(4, 1);
            joint_cmd_ = MatXf::Zero(12, 5);
            joint_cmd_.col(0) = kp_;
            joint_cmd_.col(2) = kd_;
            stand_duration_ = cp_ptr_->stand_duration_;
            time_stamp_record_ = 0.0f;
            run_time_ = 0.0f;
            previous_run_time_ = 0.0f;
            control_dt_ = 0.0f;
        }
    ~StandUpState(){}


    virtual void OnEnter() {
        GetRobotJointValue();
        RecordJointData();
        previous_run_time_ = run_time_;
        control_dt_ = 0.0f;
        last_policy_entry_gate_reason_ = PolicyEntryGateReason::kReady;
        last_policy_entry_gate_print_time_ = -10000.0f;
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::StandingUp);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        GetRobotJointValue();
        VecXf planning_joint_pos(current_joint_pos_.rows());
        VecXf planning_joint_vel(current_joint_pos_.rows());
        if(run_time_ - time_stamp_record_ <= stand_duration_){
            for(int i=0;i<current_joint_pos_.rows();++i){
                planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, stand_duration_);
                planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, stand_duration_);
            }
        }else{
            planning_joint_pos = goal_joint_pos_;
            planning_joint_vel = VecXf::Zero(current_joint_pos_.rows());
        }

        joint_cmd_.col(1) = planning_joint_pos;
        joint_cmd_.col(3) = planning_joint_vel;
        joint_cmd_ = ClampJointCommandToLimits(joint_cmd_, MiniCheetahJointCommandLimits(), 12);
        ri_ptr_->SetJointCommand(joint_cmd_); // (current torque, not last torque, video content slip of the tongue)
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        return false;
    }
    virtual StateName GetNextStateName() {
        if(run_time_ - time_stamp_record_ <= 2.*stand_duration_){
            return StateName::kStandUp;
        }else{
            if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::RLControlMode)){
                PolicyEntryGateState state;
                state.joint_pos = current_joint_pos_;
                state.joint_vel = current_joint_vel_;
                state.base_rpy = ri_ptr_->GetImuRpy();
                state.base_lin_vel = ri_ptr_->GetBaseLinearVelocity();
                state.base_height = ri_ptr_->GetBaseHeight();
                state.control_dt = control_dt_;

                const PolicyEntryGateResult gate_result =
                    EvaluatePolicyEntryGate(policy_entry_gate_config_, state);
                if(gate_result.allowed){
                    std::cout << "stand up success" << std::endl;
                    return StateName::kRLControl;
                }
                MaybePrintPolicyEntryGateRefusal(gate_result);
            }
        }
        return StateName::kStandUp;
    }
};
