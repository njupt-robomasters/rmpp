#pragma once

#include "motor/MF9025.hpp"

inline PID::config_t angle_pid = {
    .kp = (16.5 * A) / (16.5 * deg),
    .kd = (16.5 * A) / (165 * deg_s),
    .max_out = 16.5 * A,
    .fc = 10 * Hz,
};
inline MF9025 motor({
    .can_port = 1,
    .master_id = 0x141,
    .slave_id = 0x141,
    .is_invert = false,
    .offset = -115 * deg,
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &angle_pid,
});