#pragma once

#include "app/chassis.hpp"
#include "motor/gm6020.hpp"
#include "motor/m3508.hpp"

class Chassis : public Chassis_Template {
public:
    // 底盘参数
    static constexpr UnitFloat CHASSIS_RADIUS = 21.492f * cm; // 底盘半径
    static constexpr UnitFloat WHEEL_RADIUS = 5.25f * cm;     // 轮子半径

    // 舵电机偏移
    static constexpr Angle SERVO1_OFFSET = 133.33f * deg;
    static constexpr Angle SERVO2_OFFSET = 71.89f * deg;

    // 最小转舵速度
    static constexpr UnitFloat MIN_V = 0.01f * m_s;

    // 轮子线速度
    struct {
        UnitFloat<m_s> ref, measure;
    } v1, v2;

    // 舵轮角度
    struct {
        Angle<deg> measure, ref;
    } s1, s2;

    // 电机对象
    M3508 m_wheel1, m_wheel2;  // 轮电机
    GM6020 m_servo1, m_servo2; // 舵电机

    Chassis(PID::param_t* wheel_pid, PID::param_t* servo_pid, PID::param_t* follow_pid);

    // 底盘使能/失能
    void SetEnable(bool is_enable) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    // 车体速度 -> 轮子速度
    void speedForward() override;

    // 轮子速度 -> 车体速度
    void speedBackward() override;

    // 功率控制
    void powerControl() override;
};