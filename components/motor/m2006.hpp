#pragma once

#include "mdji.hpp"
#include "pid.hpp"

class M2006 : public MDJI {
public:
    explicit M2006(PID::param_t &pid_param);

    void Update();

private:
    static constexpr float CURRENT_MAX = 10.0f; // 最大电流【单位；A】
    static constexpr int16_t CURRENT_CMD_MAX = 10000; // CAN通信最大电流对应的值
    static constexpr float REDUCTION_RATIO = 36.0f; // 电机减速比
};
