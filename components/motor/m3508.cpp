#include "m3508.hpp"

M3508::M3508(PID::param_t &pid_param) : MDJI(CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION_RATIO, pid_param) {
}

void M3508::Update() {
    if (is_enable) {
        // 增量PD模式
        const float speed_err = ref.speed.tps - measure.speed.tps;
        ref.current = pid.CalcIncrement(speed_err);
    }
}

float M3508::EstimatePower() const {
    const float mechanical_power = M_PER_I * measure.current * measure.speed.rps;
    const float joule_heat = measure.current * measure.current * R;
    const float power_estimate = mechanical_power + joule_heat;
    return power_estimate;
}
