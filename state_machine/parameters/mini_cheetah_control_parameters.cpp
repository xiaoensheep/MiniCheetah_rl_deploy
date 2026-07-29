#include "control_parameters.h"

void ControlParameters::GenerateMiniCheetahParameters(){
    body_len_x_ = 0.19f * 2.f;
    body_len_y_ = 0.049f * 2.f;
    hip_len_ = 0.062f;
    thigh_len_ = 0.209f;
    shank_len_ = 0.209f;

    pre_height_ = 0.12;
    stand_height_ = 0.28;
    swing_leg_kp_ << 40., 40., 40.;
    swing_leg_kd_ << 1.0, 1.0, 1.0;

    fl_joint_lower_ << -1.60, -2.60, -2.60;
    fl_joint_upper_ <<  1.60,  2.60,  2.60;
    joint_vel_limit_ << 30, 30, 20;
    torque_limit_ << 17, 17, 26;
}
