#pragma once

#include "pid.hpp"

class Settings {
public:
    // 底盘参数
    float chassis_vx_max = 5.0f; // 左右平移速度【单位：m/s】
    float chassis_vy_max = 5.0f; // 前进后退速度【单位：m/s】
    float chassis_vr_rpm_max = 60.0f; // 旋转角速度【单位：rpm】
    // PID参数
    PID::pid_param_t wheel_pid = {10000.0f, 50.0f, 0.0f};
    PID::pid_param_t chassis_pid = {0.0f, 0.0f, 0.0f}; // 底盘速度补偿（暂时未用到，轮子PID给的激进后，小陀螺模式前进已经很稳）

    // 云台参数
    float pitch_aps_max = 30.0f; // 【单位：度/s】
    float yaw_aps_max = 60.0f; // 【单位：度/s】
    float shoot_freq = 15.0f; // 【单位：Hz】
    // PID参数
    PID::pid_param_t pitch_pid = {1.0f, 0.0f, 0.014f}; // pitch
    PID::pid_param_t yaw_pid = {0.7f, 0.0f, 0.012f}; // yaw
    PID::pid_param_t shoot_pid = {2.0f, 0.0f, 0.0f}; // shoot
};
