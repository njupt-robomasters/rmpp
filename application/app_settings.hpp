#pragma once

#include "pid.hpp"

class Settings {
public:
    // 云台PID参数
    float pitch_aps_max = 60.0f; // 【单位：度/s】
    float yaw_aps_max = 180.0f; // 【单位：度/s】
    const float mouse_x_max = 300.0f;
    const float mouse_y_max = 300.0f;
    PID::param_t wheel_pid = {.kp = 10000.0f, .ki = 50.0f};
    PID::param_t speed_comp_pid = {}; // 底盘速度补偿（暂时未用到，轮子PID给的激进后，小陀螺模式前进已经很稳）

    // PID参数
    PID::param_t pitch_pid = {.kp = 1.0f, .kd = 0.014f}; // pitch
    PID::param_t yaw_pid = {.kp = 0.7f, .kd = 0.012f}; // yaw
    PID::param_t shoot_pid = {.kp = 2.0f}; // shoot
};
