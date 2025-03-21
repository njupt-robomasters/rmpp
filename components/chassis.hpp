#pragma once

#include <cmath>
#include "m3508.hpp"
#include "utils.hpp"

class Chassis {
public:
    struct data_t {
        float ref = 0, measure = 0;
    };

    bool is_enable = false;

    // 底盘速度
    // vx 左右平移速度【单位：m/s】
    // vy 前进后退速度【单位：m/s】
    // vr 旋转角速度，逆时针为正【单位：圈/s】
    // vz 旋转线速度，逆时针为正【单位：m/s】
    data_t vx_gimbal{}, vy_gimbal{}, vz{}, vr_tps{};
    data_t vx_chassis{}, vy_chassis{};

    // 速度补偿，PID给出
    float vx_comp = 0, vy_comp = 0;

    // 原定底盘前进方向->云台方向 的角度（逆时针为正）
    float gimbal_ref_by_chassis = 0;

    explicit Chassis(const mit_t &mit);

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void SetEnable(bool is_enable);

    void SetSpeed(float vx_ref, float vy_ref, float vr_tps_ref);

    void SetGimbalRefByChassisAngle(float gimbal_ref_by_chassis);

    void Update();

private:
    // 底盘结构参数
    static constexpr float WHEEL_RADIUS = 0.063f; // 轮子半径【单位：m】
    static constexpr float CHASSIS_RADIUS = 0.225f; // 底盘半径【单位：m】

    // 底盘结构参数（自动计算）
    static constexpr float WHEEL_PERIMETER = 2 * static_cast<float>(M_PI) * WHEEL_RADIUS; // 轮子周长【单位：m】
    static constexpr float CHASSIS_PERIMETER = 2 * static_cast<float>(M_PI) * CHASSIS_RADIUS; // 底盘周长【单位：m】

    float kp = 0;

    // 轮子速度【单位：m/s】
    data_t v1{}, v2{}, v3{}, v4{};

    M3508 m1, m2, m3, m4;

    void calcSpeedComp();

    void forwardCalc();

    void inverseCalc();

    void sendCurrentCMD() const;
};
