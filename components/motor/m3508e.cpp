#include "m3508e.hpp"

M3508E::M3508E(PID::param_t &pid_param) : MDJI(CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION_RATIO, pid_param) {
}

void M3508E::Update() {
    if (is_enable) {
        // 增量PD模式
        const float speed_err = ref.speed.tps - measure.speed.tps;
        ref.current = pid.CalcIncrement(speed_err);
    }
}

float M3508E::EstimatePower() const {
    const float mechanical_power = M_PER_I * measure.current * measure.speed.rps;
    const float joule_heat = measure.current * measure.current * R;
    const float power_estimate = mechanical_power + joule_heat;
    return power_estimate;
}
