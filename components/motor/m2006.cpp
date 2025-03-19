#include "m2006.hpp"

M2006::M2006(const mit_t &mit) : MDJI(MAX_CURRENT, 10000, 36.0f),
                                 mit(mit) {
}

void M2006::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
}

void M2006::SetV_TPS(const float v_tps_ref) {
    this->v_tps_ref = v_tps_ref;
}

void M2006::Update() {
    if (is_enable) {
        // 暂时只使用P控制，后面改成增量式PD控制，并设置更小的电流钳位 todo
        const float tps_err = v_tps_ref - v_tps_lpf;
        current_ref = mit.kp * tps_err + mit.ff;
        clamp(current_ref, MAX_CURRENT); // 输出钳位
    } else {
        current_ref = 0;
    }
}
