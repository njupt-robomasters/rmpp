#pragma once

#include "motor/DM4310.hpp"

inline PID::config_t speed_pid = {
    .kp = (3 * Nm) / (150 * deg_s),
    .max_out = 3 * Nm,
    .fc = 10 * Hz,
};
inline PID::config_t angle_pid = {
    .kp = 20 * default_unit,
    .max_out = 720 * deg_s,
};
inline DM4310 motor({
    .can_port = 2,
    .master_id = 0x10,
    .slave_id = 0x11,
    .is_invert = true,
    .offset = -162 * deg,
    .is_limit = true,
    .limit_min = -35 * deg,
    .limit_max = 40 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &speed_pid,
    .angle_pid_config = &angle_pid,
});