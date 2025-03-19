#pragma once

#include "utils.hpp"

class Settings {
public:
    // 底盘参数
    float chassis_vx_mps_max = 1.0f; // 左右平移速度【单位：m/s】
    float chassis_vy_mps_max = 1.0f; // 前进后退速度【单位：m/s】
    float chassis_vr_tps_max = 1.0f; // 旋转角速度【单位：圈/s】
    // MIT格式控制参数
    mit_t chassis_mit = {50.0f, 10.0f, 0.0f};

    // 云台参数
    float pitch_aps_max = 30.0f; // 【单位：度/s】
    float yaw_tps_max = 1.0f; // 【单位：圈/s】
    float shoot_freq = 10.0f; // 【单位：Hz】
    // MIT格式控制参数
    mit_t pitch_mit = {1.0f, 0.014f, 0.4f}; // pitch
    mit_t yaw_mit = {0.7f, 0.012f, 0.0f}; // yaw
    mit_t shoot_mit = {2.0f, 0.0f, 0.5f}; // shoot
};
