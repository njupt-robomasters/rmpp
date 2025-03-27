#pragma once

#include "m3508.hpp"
#include "m6020.hpp"
#include "pid.hpp"
#include "unit.hpp"

class Chassis {
public:
    struct {
        // vx 左右平移速度【单位：m/s】
        // vy 前进后退速度【单位：m/s】
        // vz 旋转线速度，逆时针为正【单位：m/s】
        // vr 旋转角速度，逆时针为正
        struct {
            float vx = 0, vy = 0, vz = 0;
            Speed vr{};
        } chassis;

        // 轮子速度【单位：m/s】
        struct {
            float w1 = 0, w2 = 0;
            struct {
                Angle absolute{}, relative{};
            } s1{}, s2{};
        } wheel;
    } ref{}, measure{};

    Chassis(PID::param_t &servo_pid_param, PID::param_t &wheel_pid_param);

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void SetEnable(bool is_enable);

    void SetSpeed(float vx, float vy, float vr_rpm);

    void SetPowerLimit(float power);

    void Update();

private:
    // 舵电机偏移
    static constexpr float S1_OFFSET = 102.7f; //【单位：角度】
    static constexpr float S2_OFFSET = 345.0f; //【单位：角度】

    // 底盘结构参数
    static constexpr float WHEEL_RADIUS = 0.0525f; // 轮子半径【单位：m】
    static constexpr float CHASSIS_RADIUS = 0.21492f; // 底盘半径【单位：m】

    // 底盘结构参数（自动计算）
    static constexpr float WHEEL_PERIMETER = 2 * PI * WHEEL_RADIUS; // 轮子周长【单位：m】
    static constexpr float CHASSIS_PERIMETER = 2 * PI * CHASSIS_RADIUS; // 底盘周长【单位：m】

    static constexpr float ESP = 1e-3f;

    // 底盘使能标志
    bool is_enable = false;

    // 电机对象
    M6020 s1, s2; // 舵电机
    M3508 w1, w2; // 轮电机

    // 底盘功率控制
    float power_estimate = 0; // 当前功率估计
    float power_limit = 120; // 功率限制
    float current_ratio = 1; // 电流衰减系数

    void forwardCalc();

    void inverseCalc();

    void estimatePower();

    void calcCurrentRatio();

    void sendCANCmd();
};
