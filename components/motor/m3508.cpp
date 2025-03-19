#include "m3508.hpp"
#include "utils.hpp"
#include "bsp_dwt.h"

M3508::M3508(const mit_t &mit) : MDJI(CURRENT_MAX, 16384, 3591.0f / 187.0f),
                                 mit(mit) {
}

void M3508::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
}

void M3508::SetV_TPS(const float v_tps_ref) {
    this->v_tps_ref = v_tps_ref;
}

void M3508::Update() {
    dt = BSP_DWT_GetDeltaT(&dwt_cnt);

    if (is_enable) {
        // 增量式PD控制
        Pout = mit.kp * (v_tps_ref - v_tps_lpf);
        Dout = -mit.kd * a_tpss;
        current_ref += (Pout + Dout) * dt;
        clamp(current_ref, CURRENT_MAX); // 输出钳位
    } else {
        current_ref = 0;
    }
}
