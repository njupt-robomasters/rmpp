#pragma once

#include <bsp.hpp>

class PID {
public:
    struct param_t {
        float kp = 0, ki = 0, kd = 0;
        float ff = 0; // 前馈
        float max_i = 0; // 积分限制
        float max_out = 0; // 最大输出
    };

private:
    param_t default_param{};
    param_t& param = default_param; // PID参数

    float err = 0; // 误差值 err = ref - measure
    float p_out = 0, i_out = 0, d_out = 0, out_without_ff = 0; // PID输出分量
    float last_err = 0, last2_err = 0;

    float di = 0; // 当前时刻的i增量

    BSP::Dwt dwt; // 维护dt

    void calcPositionCommon();

public:
    float out = 0;

    PID() = default;

    PID(param_t& param);

    void SetParam(param_t& param);

    float CalcPosition(float err);

    float CalcPosition(float err, float derr);

    float CalcIncrement(float err);

    void Clear();
};
