#pragma once

#include "motor/DM6006.hpp"

inline PID::config_t speed_pid = {
    .kp = (4 * Nm) / (150 * deg_s),
    .max_out = 4 * Nm,
    .fc = 10 * Hz,
};
inline PID::config_t angle_pid = {
    .kp = 20 * default_unit,
    .max_out = 720 * deg_s,
};
inline DM6006 motor({
    .can_port = 2,
    .master_id = 0x12,
    .slave_id = 0x13,
    .is_invert = false,
    .offset = 172.5f * deg,
    .is_limit = true,
    .limit_min = -10 * deg,
    .limit_max = 15 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &speed_pid,
    .angle_pid_config = &angle_pid,
});