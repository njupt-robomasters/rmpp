#pragma once

#include <optional>
#include "bsp/bsp.hpp"
#include "unit/include_me.hpp"

class PID {
public:
    struct config_t {
        UnitFloat<> kp, ki, kd; // p、i、d三参数
        UnitFloat<> ff;         // 前馈
        UnitFloat<> max_i;      // 积分限幅
        UnitFloat<> max_out;    // 输出限幅
        UnitFloat<> fc;         // 输出低通滤波截止频率
    };

    UnitFloat<> out; // PID输出

    PID();

    PID(const config_t* config);

    UnitFloat<> Calculate(const UnitFloat<>& err, std::optional<UnitFloat<>> derr = std::nullopt);

    void Clear();

private:
    const config_t* config; // PID参数

    UnitFloat<> out_no_filter, p_out, i_out, d_out; // PID输出分量
    UnitFloat<> err, last_err;                      // 误差值 err = ref - measure
    BSP::Dwt dwt;                                   // 用于计算dt

    // 低通滤波
    static UnitFloat<> lowpassFilter(const UnitFloat<>& pre, const UnitFloat<>& next, const UnitFloat<>& fc, const UnitFloat<>& dt);
};