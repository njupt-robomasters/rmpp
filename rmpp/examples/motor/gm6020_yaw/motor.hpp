#pragma once

#include "motor/GM6020.hpp"

// 速度PID
inline PID::config_t speed_pid = {
    .kp = (3 * A) / (100 * deg_s),
    .ki = (3 * A) / (60 * deg),
    .max_i = 3 * A,
    .max_out = 3 * A,
    .fc = 10 * Hz,
};

// 角度PID
inline PID::config_t angle_pid = {
    .kp = 14 * default_unit,
    .max_out = 720 * deg_s,
};

// yaw电机
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