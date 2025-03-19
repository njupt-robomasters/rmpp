#pragma once

#include "utils.hpp"

class Settings {
public:
    // 底盘参数
    mit_t chassis_mit = {}
    float chassis_vx_mps_max = 5.0f; // 左右平移速度【单位：m/s】
    float chassis_vy_mps_max = 5.0f; // 前进后退速度【单位：m/s】
    float chassis_vr_tps_max = 1.0f; // 旋转角速度【单位：圈/s】
    // PID参数
    float chassis_kp = 50.0f;
    float chassis_kd = 10.0f;

    // 云台参数
    float pitch_aps_max = 30.0f; // 【单位：度/s】
    float yaw_tps_max = 1.0f; // 【单位：圈/s】
    float shoot_freq = 10.0f; // 【单位：Hz】
    // PID参数
    // pitch
    float pitch_kp = 1.0f;
    float pitch_kd = 0.014f;
    float pitch_ff = 0.4f; // 前馈
    // yaw
    float yaw_kp = 0.7f;
    float yaw_kd = 0.012f;
    // shoot
    float shoot_kp = 2.0f;
    float shoot_ff = 0.5f;
};
