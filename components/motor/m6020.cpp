#include "m6020.hpp"
#include "utils.hpp"

M6020::M6020(const float Kp, const float Kd, const float feedforward) : MDJI(MAX_CURRENT, 16384, 1) {
    this->Kp = Kp;
    this->Kd = Kd;
    this->feedforward = feedforward;
}

void M6020::Update(const float angle_ref) {
    this->angle_ref = angle_ref;

    const float angle_err = calc_angle_err(angle_ref, angle);

    // MIT控制模式
    Pout = Kp * signed_sqrt(angle_err);
    Dout = -Kd * speed_aps_lpf;
    current_ref = Pout + Dout + feedforward;

    // 输出钳位
    clamp(current_ref, MAX_CURRENT);
}
