#pragma once

#include "unit.hpp"
#include "pid.hpp"
#include "m6020.hpp"
#include "m3508.hpp"

extern "C" void task_can_entry(const void* argument);

class Chassis {
    friend void task_can_entry(const void* argument);

private:
    // 底盘参数
    static constexpr UnitFloat<m> WHEEL_RADIUS = 0.0525f * m; // 底盘半径
    static constexpr UnitFloat<m> CHASSIS_RADIUS = 0.21492f * m; // 轮子半径

    // 舵电机偏移
    static constexpr Angle<deg> SERVO1_OFFSET = -136.0f * deg;
    static constexpr Angle<deg> SERVO2_OFFSET = -102.0f * deg;

    // 最小转舵速度
    static constexpr UnitFloat<m_s> MIN_V = 1e-2f;

    // 电机对象
    M6020 m_servo1, m_servo2; // 舵电机
    M3508 m_wheel1, m_wheel2; // 轮电机

    // 底盘使能标志
    bool is_enable = false;

    // 云台yaw方向
    Angle<deg> gimbal_yaw;

    void forwardCalc();

    void backwardCalc();

    void estimatePower();

    void calcCurrentRatio();

public:
    // 底盘速度
    // vx 前后速度，前为正
    // vy 左右速度，左为正
    // vz 旋转线速度，逆时针为正
    // vr 旋转角速度，逆时针为正
    struct {
        struct {
            UnitFloat<m_s> ref, measure;
        } chassis, gimbal;
    } vx, vy;
    struct {
        UnitFloat<m_s> ref, measure;
    } vz;
    struct {
        UnitFloat<rpm> ref, measure;
    } vr;

    // 轮子线速度
    struct {
        UnitFloat<m_s> ref, measure;
    } v1, v2;

    // 舵轮角度
    struct {
        struct {
            Angle<deg> measure, ref;
        } relative, absolute;
    } s1, s2;

    // 底盘功率控制
    struct {
        UnitFloat<W> estimate = 0; // 当前功率估计
        UnitFloat<W> limit = 120;
        float current_ratio = 1; // 电流衰减系数
    } power;

    Chassis(PID::param_t* servo_pid_param, PID::param_t* wheel_pid_param);

    void SetEnable(bool is_enable);

    void SetGimbalYaw(const Angle<>& gimbal_yaw);

    void SetSpeed(const UnitFloat<>& vx, const UnitFloat<>& vy, const UnitFloat<>& vr);

    void SetPowerLimit(const UnitFloat<>& power);

    void Update();
};
