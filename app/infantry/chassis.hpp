#pragma once

#include "app/chassis.hpp"
#include "motor/m3508.hpp"

class Chassis : public Chassis_Template {
public:
    // 底盘参数
    static constexpr UnitFloat CHASSIS_RADIUS = 19.80f * cm; // 底盘半径
    static constexpr UnitFloat WHEEL_RADIUS = 7.65f * cm;    // 轮子半径

    // 轮子线速度
    struct {
        UnitFloat<m_s> ref, measure;
    } v1, v2, v3, v4;

    // 电机对象
    M3508 m_wheel1, m_wheel2, m_wheel3, m_wheel4;

    Chassis(PID::param_t* wheel_pid, PID::param_t* follow_pid);

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