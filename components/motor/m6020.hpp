#pragma once

#include "mdji.hpp"
#include "utils.hpp"

class M6020 : public MDJI {
public:
    explicit M6020(const mit_t &mit);

    void SetEnable(bool is_enable);

    void SetAngle(float angle_ref, float v_aps_ff = 0);

    void Update();

private:
    static constexpr float CURRENT_MAX = 3.0f; // 最大电流【单位；A】
    static constexpr int16_t CURRENT_CMD_MAX = 16384; // CAN通信最大电流对应的值
    static constexpr float REDUCTION_RATIO = 1.0f; // 电机减速比

    // MIT格式控制参数
    const mit_t &mit;

    bool is_enable = false;

    float angle_ref = 0; // 目标角度【单位：角度】
    float v_ff_aps = 0; // 速度前馈

    float Pout = 0, Dout = 0;
};
