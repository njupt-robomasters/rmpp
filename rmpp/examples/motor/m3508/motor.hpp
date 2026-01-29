#pragma once

#include "motor/M3508.hpp"

// 速度PID参数
inline PID::config_t wheel_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (20 * A) / (120 * rpm),
    .ki = (20 * A) / (15 * deg),
    .max_i = 20 * A,
    .max_out = 20 * A,
};

// 电机
inline M3508 motor({
    .can_port = 1,
    .master_id = 0x201,
    .slave_id = 0x200,
    .reduction = 268.0f / 17.0f,
    .Kt = M3508::Kt,
    .R = M3508::R,
    .is_invert = false,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid,
});