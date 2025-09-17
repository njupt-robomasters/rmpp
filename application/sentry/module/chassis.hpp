#pragma once

#include "m6020.hpp"
#include "m3508.hpp"
#include "pid.hpp"
#include "unit.hpp"

class Chassis {
private:
    // 舵电机偏移
    static constexpr Angle<deg> SERVO1_OFFSET = -136.0f * deg;
    static constexpr Angle<deg> SERVO2_OFFSET = -102.0f * deg;

    // 底盘参数
    static constexpr Unit<m> WHEEL_RADIUS = 0.0525f * m; // 底盘半径
    static constexpr Unit<m> CHASSIS_RADIUS = 0.21492f * m; // 轮子半径

    static constexpr Unit<m_s> MIN_V = 1e-2f; // 最小转舵速度

    // 电机对象
    M6020 m6020_1, m6020_2; // 舵电机
    M3508 m3508_1, m3508_2; // 轮电机

    // 底盘使能标志
    bool is_enable = false;

    // 底盘功率控制
    struct {
        Unit<W> estimate = 0; // 当前功率估计
        Unit<W> limit = 120;
        float current_ratio = 1; // 电流衰减系数
    } power{};

    void forwardCalc();

    void inverseCalc();

    void estimatePower();

    void calcCurrentRatio();

    void sendCANCmd();

public:
    // vx 前后速度，前为正
    // vy 左右速度，左为正
    // vz 旋转线速度，逆时针为正
    // vr 旋转角速度，逆时针为正
    struct {
        Unit<m_s> ref = 0, measure = 0;
    } vx{}, vy{}, vz{};
    struct {
        Unit<rpm> ref = 0, measure = 0;
    } vr{};

    // 轮子线速度
    struct {
        Unit<m_s> ref = 0, measure = 0;
    } v1{}, v2{};

    // 舵轮角度
    struct {
        struct {
            Angle<deg> measure, ref;
        } relative{}, absolute{};
    } s1{}, s2{};

    Chassis(PID::param_t& servo_pid_param, PID::param_t& wheel_pid_param);

    void SetEnable(bool is_enable);

    void SetSpeed(const Unit<m_s>& vx, const Unit<m_s>& vy, const Unit<rpm>& vr);

    void SetPowerLimit(const Unit<W>& power);

    void Update();
};
