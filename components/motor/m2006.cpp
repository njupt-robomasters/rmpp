#include "m2006.hpp"
#include "utils.hpp"

M2006::M2006(const float Kp, const float feedforward) : MDJI(MAX_CURRENT, 10000, 36.0f) {
    this->Kp = Kp;
    this->feedforward = feedforward;
}

void M2006::Update(const float speed_tps_ref) {
    this->speed_tps_ref = speed_tps_ref;

    const float err = speed_tps_ref - speed_tps_lpf;
    current_ref = Kp * err + feedforward;

    clamp(current_ref, MAX_CURRENT);
}
