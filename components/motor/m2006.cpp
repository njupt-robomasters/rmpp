#include "m2006.hpp"

M2006::M2006(const mit_t &mit) : MDJI(CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION_RATIO),
                                 mit(mit) {
}

void M2006::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
}

void M2006::SetV(const float v_ref_tps, const float a_ff_tpss) {
    this->v_ref_tps = v_ref_tps;
    this->a_ff_tpss = a_ff_tpss;
}

void M2006::Update() {
    if (is_enable) {
        // 暂时只使用P控制，后面改成增量式PD控制，并设置更小的电流钳位 todo
        const float tps_err = v_ref_tps - v_measure_lpf_tps;
        current_ref = mit.kp * tps_err + mit.ff;
        clamp(current_ref, CURRENT_MAX); // 输出钳位
    } else {
        current_ref = 0;
    }
}
