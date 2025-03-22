#include "m3508.hpp"

M3508::M3508(const PID::pid_param_t &pid_param) : MDJI(CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION_RATIO),
                                                  pid(pid_param, CURRENT_MAX) {
}

void M3508::ParseCAN(const uint8_t data[8]) {
    MDJI::ParseCAN(data);
    powerEstimate();
}

void M3508::Update() {
    if (is_enable) {
        // 增量PD模式
        const float speed_err = ref.speed.tps - measure.speed.tps;
        ref.current = pid.CalcIncrement(speed_err);
    } else {
        ref.current = 0;
        pid.Clear();
    }
}

void M3508::powerEstimate() {
    const float mechanical_power = M_PER_I * measure.current * measure.speed.rps;
    const float joule_heat = measure.current * measure.current * R;
    power_estimate = mechanical_power + joule_heat;
}
