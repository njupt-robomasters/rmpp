#include "m3508.hpp"
#include "utils.hpp"
#include "bsp_dwt.h"

M3508::M3508(const mit_t &mit) : MDJI(CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION_RATIO),
                                 mit(mit) {
}

void M3508::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
}

void M3508::SetV(const float v_ref_tps, const float a_ff_tpss) {
    this->v_ref_tps = v_ref_tps;
    this->a_ff_tpss = a_ff_tpss;
}

void M3508::Update() {
    dt = BSP_DWT_GetDeltaT(&dwt_cnt);

    if (is_enable) {
        // 增量式PD控制
        Pout = mit.kp * (v_ref_tps - v_measure_lpf_tps);
        Dout = mit.kd * (a_ff_tpss - a_measure_tpss);
        current_ref += (Pout + Dout) * dt;
        clamp(current_ref, CURRENT_MAX); // 输出钳位
    } else {
        current_ref = 0;
    }
}
