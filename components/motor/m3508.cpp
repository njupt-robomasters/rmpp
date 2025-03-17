#include "m3508.hpp"
#include "utils.hpp"

M3508::M3508(const float Kp, const float Ki, const float Imax) : MDJI(MAX_CURRENT, 16384, 3591.0f / 187.0f) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Imax = Imax;
}

void M3508::Update(const float speed_rpm_ref) {
    this->speed_rpm_ref = speed_rpm_ref;

    const float err = speed_rpm_ref - speed_rpm_lpf;
    current_ref = Kp * err;

    clamp(current_ref, MAX_CURRENT);
}
