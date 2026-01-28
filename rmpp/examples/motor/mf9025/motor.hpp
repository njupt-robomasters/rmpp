#pragma once

#include "motor/MF9025.hpp"

// 角度PID
inline PID::config_t angle_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (10 * A) / (50 * deg),
    .kd = (10 * A) / (500 * deg_s),
    .max_out = 10 * A,
    .fc = 10 * Hz,
};

inline MF9025 motor({
    .can_port = 1,
    .master_id = 0x141,
    .slave_id = 0x141,
    .reduction = MF9025::reduction,
    .Kt = MF9025::Kt,
    .is_invert = false,
    .offset = 102.71 * deg,
    .control_mode = Motor::ANGLE_MODE,
    .speed_pid_output = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &angle_pid,
});