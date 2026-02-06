#pragma once

#include "motor/GM6020.hpp"

inline PID::config_t speed_pid = {
    .kp = (3 * A) / (300 * deg_s),
    .kd = (3 * A) / (30000 * deg_ss),
    .max_out = 3 * A,
    .fc = 10 * Hz,
};
inline PID::config_t angle_pid = {
    .kp = 30 * default_unit,
    .max_out = 720 * deg_s,
};
inline GM6020 motor({
    .can_port = 1,
    .master_id = 0x205,
    .slave_id = 0x1FF,
    .offset = -120 * deg,
    .is_limit = true,
    .limit_min = -90 * deg,
    .limit_max = 90 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &speed_pid,
    .angle_pid_config = &angle_pid,
});