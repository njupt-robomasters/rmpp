#pragma once

#include "Chassis.hpp"
#include "motor/Motor.hpp"

class Chassis_Omni : public Chassis {
public:
    // 底盘电机
    struct motor_t {
        Motor &w1, &w2, &w3, &w4; // 电机（左前，左后，右后，右前）
    } motor;

    Chassis_Omni(const config_t& config, const motor_t& motor, const PID::config_t* vxyz_pid_config);

    // 设置底盘使能/失能
    void SetEnable(bool is_enable) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    PID vx_pid, vy_pid, vz_pid; // 底盘运动PID控制器

    // 速度、力学正解
    void forward() override;

    // 速度、力学逆解
    void backward() override;

    // 底盘运动PID
    void handleXYZPID();

    // 功率控制，必须放在逆解【之前】
    void powerControl();
};