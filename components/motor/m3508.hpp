#pragma once

#include "mdji.hpp"
#include "pid.hpp"

class M3508 : public MDJI {
public:
    // 用于底盘功率估计
    static constexpr float R = 0.194f / 2; // 相电阻（两相电阻/2）
    float kt = 0.55f; // 输出轴力矩常数

    M3508(uint8_t can_port, uint32_t feedback_can_id);

    void Update();

    void SetKt(float kt);

    float EstimatePower();

private:
    static constexpr Unit<A> CURRENT_MAX = 20.0f;
    static constexpr int16_t CURRENT_CMD_MAX = 16384;
    static constexpr float REDUCTION = 3591.0f / 187.0f;
};
