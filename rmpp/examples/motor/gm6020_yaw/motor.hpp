#pragma once

#include "motor/GM6020.hpp"

inline PID::config_t speed_pid = {
    .kp = 0.04f * (A / deg_s),
    .ki = 0.05f * (A / deg),
    .max_i = 3 * A,
    .max_out = 3 * A,
    .fc = 10 * Hz,
};
inline PID::config_t angle_pid = {
    .kp = 30 * default_unit,
    .kd = 1 * default_unit,
    .max_out = 720 * deg_s,
};
inline GM6020 motor({
    .can_port = 1,
    .master_id = 0x205,
    .slave_id = 0x1FF,
    .is_invert = false,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &speed_pid,
    .angle_pid_config = &angle_pid,
});