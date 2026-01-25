#pragma once

#include "Chassis.hpp"
#include "motor/Motor.hpp"

class Chassis_Omni : public Chassis {
public:
    // 底盘电机
    struct motor_t {
        Motor &w1, &w2, &w3, &w4; // 电机（左前，左后，右后，右前）
    } motor;

    // 轮子线速度
    struct {
        UnitFloat<m_s> ref, measure;
    } v1, v2, v3, v4;

    Chassis_Omni(const config_t& config, const motor_t& motor);

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