#pragma once

#include "mdji.hpp"
#include "pid.hpp"

class M3508 : public MDJI {
public:
    float estimate_power = 0;

    explicit M3508(const PID::pid_param_t &pid_param);

    void Update();

    void EstimatePower();

private:
    static constexpr float CURRENT_MAX = 20.0f; // 最大电流【单位；A】
    static constexpr int16_t CURRENT_CMD_MAX = 16384; // CAN通信最大电流对应的值
    static constexpr float REDUCTION_RATIO = 3591.0f / 187.0f; // 电机减速比

    // 用于底盘功率估计
    static constexpr float M_PER_I = 1.5785e-2; // 电流 -> 力矩 系数（转子轴）
    static constexpr float R = 82.5e-3; // 电机内阻

    PID pid;
};
