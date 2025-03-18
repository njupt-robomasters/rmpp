#pragma once

class Settings {
public:
    // 底盘运动参数
    float chassis_vx_max = 5.0f; // 左右平移速度【单位：m/s】
    float chassis_vy_max = 5.0f; // 前进后退速度【单位：m/s】
    float chassis_vr_max = 60.0f; // 旋转角速度【单位：rpm】

    // 云台运动参数
    float pitch_max_speed = 30.0f; // 【单位：度/s】
    float yaw_max_speed = 360.0f; // 【单位：度/s】
    float shoot_freq = 10.0f; // 【单位：Hz】
};
