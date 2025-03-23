#pragma once

#include "gimbal.hpp"
#include "pid.hpp"

class Settings {
public:
    // 底盘参数
    float chassis_vxy_max = 1.0f; // 前后左右平移速度【单位：m/s】
    float chassis_vr_max_rpm = 100.0f; // 旋转角速度【单位：rpm】
    // PID参数
    PID::param_t wheel_pid = {.kp = 10000.0f, .ki = 50.0f};
    PID::param_t speed_comp_pid = {}; // 底盘速度补偿（暂时未用到，轮子PID给的激进后，小陀螺模式前进已经很稳）

    // 云台参数
    Gimbal::mode_e gimbal_mode = Gimbal::IMU_MODE;
    float pitch_aps_max = 30.0f; // 【单位：度/s】
    float yaw_aps_max = 60.0f; // 【单位：度/s】
    float shoot_freq = 15.0f; // 【单位：Hz】
    // PID参数
    PID::param_t pitch_pid = {.kp = 1.0f, .kd = 0.014f}; // pitch
    PID::param_t yaw_pid = {.kp = 0.7f, .kd = 0.012f}; // yaw
    PID::param_t shoot_pid = {.kp = 2.0f}; // shoot
};
