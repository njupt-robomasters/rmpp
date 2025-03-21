#pragma once

#include "mdji.hpp"
#include "pid.hpp"

class M6020 : public MDJI {
public:
    explicit M6020(const PID::pid_param_t &pid_param);

    void Update();

private:
    static constexpr float CURRENT_MAX = 3.0f; // 最大电流【单位；A】
    static constexpr int16_t CURRENT_CMD_MAX = 16384; // CAN通信最大电流对应的值
    static constexpr float REDUCTION_RATIO = 1.0f; // 电机减速比

    PID pid;
};
