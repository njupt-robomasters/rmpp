#include "m6020.hpp"
#include "utils.hpp"

M6020::M6020(const mit_t &mit) : MDJI(CURRENT_MAX, 16384, 1),
                                 mit(mit) {
}

void M6020::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
}

void M6020::SetAngle(const float angle_ref) {
    this->angle_ref = angle_ref;
}

void M6020::SetV_APS_FF(const float v_aps_ff) {
    this->v_aps_ff = v_aps_ff;
}

void M6020::Update() {
    if (is_enable) {
        // MIT控制模式（ZCY优化版）
        const float angle_err = calc_angle_err(angle_ref, angle);
        Pout = mit.kp * signed_sqrt(angle_err);
        Dout = mit.kd * (v_aps_ff - v_aps_lpf);
        current_ref = Pout + Dout + mit.ff;
        clamp(current_ref, CURRENT_MAX); // 输出钳位
    } else {
        current_ref = 0;
    }
}
