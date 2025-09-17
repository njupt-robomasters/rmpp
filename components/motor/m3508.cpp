#include "m3508.hpp"

M3508::M3508(uint8_t can_port, uint32_t feedback_can_id) :
    MDJI(can_port, feedback_can_id,
         CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION),
    kt(kt) {}

void M3508::Update() {
    if (is_enable) {
        const float speed_err = speed.ref - speed.measure;
        current.ref = pid.CalcIncrement(speed_err);
    } else {
        pid.Clear();
        current.ref = 0;
    }
}

void M3508::SetKt(float kt) {
    this->kt = kt;
}

float M3508::EstimatePower() {
    const float mechanical_power = kt * current.measure * speed.measure; // 机械功率
    const float joule_heat = 3 * current.measure * current.measure * R; // 焦耳热功率
    const float power_estimate = mechanical_power + joule_heat; // 总功率估计
    return power_estimate;
}
