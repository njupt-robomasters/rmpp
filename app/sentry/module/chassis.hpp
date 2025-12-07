#pragma once

#include "template/chassis.hpp"
#include "motor/gm6020.hpp"
#include "motor/m3508.hpp"

class Chassis : public Chassis_Template {
public:
    // 底盘参数
    static constexpr UnitFloat CHASSIS_RADIUS = 21.492f * cm; // 底盘半径
    static constexpr UnitFloat WHEEL_RADIUS = 5.25f * cm;     // 轮子半径

    // 舵电机偏移
    static constexpr Angle SERVO1_OFFSET = -134.82f * deg;
    static constexpr Angle SERVO2_OFFSET = -77.65f * deg;

    // 最小转舵速度
    static constexpr UnitFloat MIN_V = 0.01f * m_s;

    // 用于功率控制
    static constexpr float R = 0.194f / 2; // 相电阻（两相电阻/2）
    static constexpr float Kt = 0.55f;     // 输出轴力矩常数

    // 轮子线速度
    struct {
        UnitFloat<m_s> ref, measure;
    } v1, v2;

    // 舵轮角度
    struct {
        Angle<deg> measure, ref;
    } s1, s2;

    // 电机对象
    GM6020 m_servo1, m_servo2; // 舵电机
    M3508 m_wheel1, m_wheel2;  // 轮电机

    Chassis(PID::param_t* servo_pid_param, PID::param_t* wheel_pid_param);

    // 底盘使能/失能
    void SetEnable(bool is_enable) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    // 车体速度 -> 轮子速度
    void forwardCalc() override;

    // 轮子速度 -> 车体速度
    void backwardCalc() override;

    // 功率控制
    void powerControl() override;
};
