#pragma once

#include "gimbal.hpp"

class Status {
public:
    // 底盘
    float chassis_vxy_max = 2.0f; // 前后左右平移速度【单位：m/s】
    float chassis_avy = 0.5f; // 平移方向加速度【单位：m/s^2】
    float chassis_vr_rpm = 10.0f; // 旋转角速度【单位：rpm】

    // 云台
    Gimbal::mode_e gimbal_mode = Gimbal::IMU_MODE;
    bool is_prepare_shoot = false;
    bool is_shoot = false;
    bool is_rv2_mode = false;
    float shoot_freq = 15.0f; // 【单位：Hz】
};
