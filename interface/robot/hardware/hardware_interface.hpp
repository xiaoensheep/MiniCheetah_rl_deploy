#pragma once
#include "robot_interface.h"

class HardwareInterface : public RobotInterface
{
private:
    Vec3f omega_body_, rpy_, acc_;
    VecXf joint_pos_, joint_vel_, joint_tau_;
public:
    explicit HardwareInterface(const std::string& robot_name):RobotInterface(robot_name, 12){
        std::cout << robot_name << " is using Mini Cheetah hardware bridge placeholder" << std::endl;
        joint_pos_ = VecXf::Zero(dof_num_);
        joint_vel_ = VecXf::Zero(dof_num_);
        joint_tau_ = VecXf::Zero(dof_num_);
        joint_cmd_ = MatXf::Zero(dof_num_, 5);
        omega_body_.setZero();
        rpy_.setZero();
        acc_.setZero();
    }
    ~HardwareInterface(){}

    virtual void Start(){
        std::cerr << "[MiniCheetah HardwareInterface] Real hardware bridge is not wired yet.\n"
                  << "Use -DBUILD_SIM=ON for sim2sim, or implement this class against "
                  << "Cheetah-Software SPI/LCM/RobotController interfaces before running on hardware."
                  << std::endl;
        start_flag_ = false;
    }

    virtual void Stop(){
    }

    virtual double GetInterfaceTimeStamp(){
        return 0.0;
    }
    virtual VecXf GetJointPosition() {
        return joint_pos_;
    };
    virtual VecXf GetJointVelocity() {
        return joint_vel_;
    }
    virtual VecXf GetJointTorque() {
        return joint_tau_;
    }
    virtual Vec3f GetImuRpy() {
        return rpy_;
    }
    virtual Vec3f GetImuAcc() {
        return acc_;
    }
    virtual Vec3f GetImuOmega() {
        return omega_body_;
    }
    virtual VecXf GetContactForce() {
        return VecXf::Zero(4);
    }
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input){
        joint_cmd_ = input;
    }
};

