#pragma once

#include "Chassis.hpp"
#include "motor/Motor.hpp"

class Chassis_Mecanum : public Chassis {
public:
    // 底盘电机
    struct motor_t {
        Motor &w1, &w2, &w3, &w4; // 电机（左前，左后，右后，右前）
    } motor;

    Chassis_Mecanum(const config_t& config, const motor_t& motor);

    // 设置底盘使能/失能
    void SetEnable(bool is_enable) override;

    // 在自定义UI上显示电机连接状态
    void HandleUI(UI& ui) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    // 速度和力学正解
    void forward() override;

    // 速度和力学逆解
    void backward() override;
};