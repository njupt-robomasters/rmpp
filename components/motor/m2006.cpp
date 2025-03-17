#include "m2006.hpp"
#include "utils.hpp"

M2006::M2006(const float Kp, const float Ki, const float Imax) : MDJI(MAX_CURRENT, 10000, 36.0f) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Imax = Imax;
}

void M2006::Update(const float speed_tps_ref) {
    this->speed_tps_ref = speed_tps_ref;

    const float err = speed_tps_ref - speed_tps;
    current_ref = Kp * err;

    clamp(current_ref, MAX_CURRENT);
}
