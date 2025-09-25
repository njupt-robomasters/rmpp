#pragma once

#include "bsp.hpp"
#include "unit.hpp"

class PID {
public:
    struct param_t {
        UnitFloat<> kp, ki, kd;
        UnitFloat<> ff; // 前馈
        UnitFloat<> max_i; // 积分限制
        UnitFloat<> max_out; // 最大输出
    };

private:
    param_t* param = nullptr; // PID参数

    float err = 0; // 误差值 err = ref - measure
    float p_out = 0, i_out = 0, d_out = 0, out_without_ff = 0; // PID输出分量
    float last_err = 0, last2_err = 0;
    float di = 0; // 当前时刻的i增量

    BSP::Dwt dwt; // 用于计算dt

    void calcPositionCommon();

public:
    float out = 0;

    PID() = default;

    PID(param_t* param);

    void SetParam(param_t* param);

    float CalcMIT(float err);

    float CalcMIT(float err, float derr);

    float CalcIncrement(float err);

    void Clear();
};
