#pragma once

#include <cmath>
#include "m3508.hpp"
#include "utils.hpp"

class Chassis {
public:
    bool is_enable = false;

    // vx 左右平移速度【单位：m/s】
    // vy 前进后退速度【单位：m/s】
    // vr 旋转角速度，逆时针为正【单位：圈/s】
    // vz 旋转线速度，逆时针为正【单位：m/s】

    // 底盘设定速度
    float vx_set_mps = 0, vy_set_mps = 0, vr_set_tps = 0, vz_set_mps = 0;
    float vx_rotate_set_mps = 0, vy_rotate_set_mps = 0;

    // 底盘实际速度（逆运动学求得）
    float vx_measure_mps = 0, vy_measure_mps = 0, vr_measure_tps = 0, vz_measure_mps = 0;
    float vx_rotate_measure_mps = 0, vy_rotate_measure_mps = 0;

    // 原定底盘前进方向->云台方向 的角度（逆时针为正）
    float forward_angle = 0;

    explicit Chassis(const mit_t &mit);

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void SetEnable(bool is_enable);

    void SetSpeed(float vx_set_mps, float vy_set_mps, float vr_set_tps);

    void SetForwardAngle(float forward_angle);

    void Update();

private:
    // 底盘结构参数
    static constexpr float WHEEL_RADIUS = 0.063f; // 轮子半径【单位：m】
    static constexpr float CHASSIS_RADIUS = 0.225f; // 底盘半径【单位：m】

    // 底盘结构参数（自动计算）
    static constexpr float WHEEL_PERIMETER = 2 * static_cast<float>(M_PI) * WHEEL_RADIUS; // 轮子周长【单位：m】
    static constexpr float CHASSIS_PERIMETER = 2 * static_cast<float>(M_PI) * CHASSIS_RADIUS; // 底盘周长【单位：m】

    // 轮子设定速度【单位：m/s】
    float v1_mps_set = 0, v2_mps_set = 0, v3_mps_set = 0, v4_mps_set = 0;
    // 小陀螺模式前馈加速度【单位：rad/s^2】
    float v1_a_ff = 0, v2_a_ff = 0, v3_a_ff = 0, v4_a_ff = 0;

    // 轮子实际速度【单位：m/s】
    float v1_mps = 0, v2_mps = 0, v3_mps = 0, v4_mps = 0;

    float d_vx_rotate = 0, d_vy_rotate;

    M3508 m1, m2, m3, m4;

    void forwardCalc();

    void inverseCalc();

    void sendCurrentCMD() const;
};
