#pragma once

#include <cmath>
#include "m3508.hpp"

class Chassis {
public:
    // vx 左右平移速度【单位：m/s】
    // vy 前进后退速度【单位：m/s】
    // vr 旋转角速度，逆时针为正【单位：rpm】
    // vz 旋转线速度，逆时针为正【单位：m/s】

    // 底盘设定速度
    float vx_set = 0, vy_set = 0, vr_set = 0, vz_set = 0;

    // 底盘实际速度（逆运动学求得）
    float vx = 0, vy = 0, vr = 0, vz = 0;

    Chassis();

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void Update(float vx_set, float vy_set, float vr_set);

    void Release();

private:
    // 底盘结构参数
    static constexpr float WHEEL_RADIUS = 0.063f; // 轮子半径【单位：m】
    static constexpr float CHASSIS_RADIUS = 0.465f; // 底盘半径【单位：m】

    // 底盘结构参数（自动计算）
    static constexpr float WHEEL_PERIMETER = 2 * static_cast<float>(M_PI) * WHEEL_RADIUS; // 轮子周长【单位：m】
    static constexpr float CHASSIS_PERIMETER = 2 * static_cast<float>(M_PI) * CHASSIS_RADIUS; // 底盘周长【单位：m】

    // PID参数
    static constexpr float P_MAX_M_S = 1.0f; // 误差达到此值后，P输出拉满【单位：轮子线速度m/s】
    static constexpr float KP = M3508::MAX_CURRENT / (P_MAX_M_S / WHEEL_PERIMETER * 60.0f);
    static constexpr float KI = 0.0f;
    static constexpr float IMAX = 0.0f;

    // 轮子设定速度【单位：m/s】
    float v1_set = 0, v2_set = 0, v3_set = 0, v4_set = 0;

    // 轮子实际速度【单位：m/s】
    float v1 = 0, v2 = 0, v3 = 0, v4 = 0;

    M3508 m1, m2, m3, m4;

    void forwardCalc();

    void inverseCalc();

    void sendCANCmd() const;
};
