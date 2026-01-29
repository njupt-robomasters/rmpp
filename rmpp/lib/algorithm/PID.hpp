#pragma once

#include <optional>
#include "bsp/bsp.hpp"
#include "unit/include_me.hpp"

class PID {
public:
    enum mode_e {
        POSITION_MODE,
        INCREMENT_MODE, // 增量式PID模式下，积分限幅、前馈、derr无效
    };

    struct config_t {
        mode_e mode = POSITION_MODE;
        UnitFloat<> kp, ki, kd; // p、i、d三参数
        UnitFloat<> ff;         // 前馈
        UnitFloat<> max_i;      // 积分限幅
        UnitFloat<> max_out;    // 输出限幅
        UnitFloat<> fc;         // 输出低通截止频率
    };

    UnitFloat<> out; // PID输出

    PID();

    PID(const config_t* config);

    UnitFloat<> Calculate(const UnitFloat<>& err, std::optional<UnitFloat<>> derr = std::nullopt);

    void Clear();

private:
    const config_t* config; // PID参数

    UnitFloat<> err, last_err, last_err2;           // 误差值 err = ref - measure
    UnitFloat<> p_out, i_out, d_out, out_no_filter; // pid输出

    BSP::Dwt dwt; // 用于计算dt

    // 经典PID
    void calcPosition(const UnitFloat<>& err, const std::optional<UnitFloat<>>& derr = std::nullopt);

    // 增量PID
    void calcIncrement(const UnitFloat<>& err);

    // 低通滤波
    static UnitFloat<> lowpassFilter(const UnitFloat<>& pre, const UnitFloat<>& next, const UnitFloat<>& fc, const UnitFloat<>& dt);
};