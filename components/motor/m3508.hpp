#pragma once

#include "mdji.hpp"
#include "utils.hpp"

class M3508 : public MDJI {
public:
    explicit M3508(const mit_t &mit);

    void SetEnable(bool is_enable);

    void SetV(float v_ref_tps, float a_ff_tpss = 0);

    void Update();

private:
    static constexpr float CURRENT_MAX = 20.0f; // 最大电流【单位；A】
    static constexpr int16_t CURRENT_CMD_MAX = 16384; // CAN通信最大电流对应的值
    static constexpr float REDUCTION_RATIO = 3591.0f / 187.0f; // 电机减速比

    // MIT格式控制参数
    const mit_t &mit;

    bool is_enable = false;

    float v_ref_tps = 0; // 目标速度（减速后）【单位：圈/s】
    float a_ff_tpss = 0; // 加速度前馈【单位：圈/s^2】

    float Pout = 0, Dout = 0;

    // 用于增量式PD控制
    uint32_t dwt_cnt = 0;
    float dt = 0;
};
