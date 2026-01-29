#pragma once

#include "motor/M2006.hpp"

// 速度PID参数
inline PID::config_t wheel_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (10 * A) / (20 * rpm),
    .max_out = 10 * A,
};

// 电机
inline M2006 motor({
    .can_port = 1,
    .master_id = 0x207,
    .slave_id = 0x200,
    .reduction = M2006::reduction,
    .Kt = M2006::Kt,
    .R = M2006::R,
    .is_invert = false,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid,
});