#include "m6020.hpp"
#include "utils.hpp"

M6020::M6020(const PID::pid_param_t &pid_param) : MDJI(CURRENT_MAX, CURRENT_CMD_MAX, REDUCTION_RATIO),
                                                  pid(pid_param, CURRENT_MAX) {
}

void M6020::Update() {
    if (is_enable) {
        // pid控制模式（ZCY优化版）
        const float angle_err = calc_angle_err(ref.angle.degree, measure.angle.degree);
        const float angle_err_sqrt = signed_sqrt(angle_err);
        const float speed_err = ref.speed.aps - measure.speed.aps;
        ref.current = pid.CalcPosition(angle_err_sqrt, speed_err);
    } else {
        ref.current = 0;
        pid.Clear();
    }
}
