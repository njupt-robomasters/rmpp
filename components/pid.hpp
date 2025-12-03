#pragma once

#include "bsp.hpp"
#include "unit/include_me.hpp"

class PID {
public:
    struct param_t {
        UnitFloat<> kp, ki, kd; // p、i、d三参数
        UnitFloat<> ff;         // 前馈
        UnitFloat<> max_i;      // 积分限幅
        UnitFloat<> max_out;    // 输出限幅
        UnitFloat<> fc;         // 输入低通截止频率
    };

    param_t* param = nullptr; // PID参数

    UnitFloat<> err, last_err, last_err2; // 误差值 err = ref - measure
    UnitFloat<> di;                       // 当前时刻的i增量
    UnitFloat<> out, p_out, i_out, d_out; // pid输出

    BSP::Dwt dwt; // 用于计算dt

    PID() = default;

    PID(param_t* param);

    void SetParam(param_t* param);

    // 经典PID
    UnitFloat<> CalcPosition(const UnitFloat<>& err);

    // MIT控制模式
    UnitFloat<> CalcPosition(const UnitFloat<>& err, const UnitFloat<>& derr);

    // 增量PID
    UnitFloat<> CalcIncrement(const UnitFloat<>& err);

    void Clear();

private:
    // 低通滤波
    static UnitFloat<> lowpassFilter(const UnitFloat<>& pre, const UnitFloat<>& next, const UnitFloat<>& fc, float dt);
};
