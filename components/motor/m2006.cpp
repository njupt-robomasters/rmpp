#include "m2006.hpp"

M2006::M2006(PID::param_t &pid_param) : MDJI(CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION_RATIO, pid_param) {
}

void M2006::Update() {
    if (is_enable) {
        // todo 暂时只使用P控制，后面改成增量式PD控制，并设置更小的电流钳位
        const float speed_err = ref.speed.tps - measure.speed.tps;
        ref.current = pid.CalcPosition(speed_err);
    }
}
