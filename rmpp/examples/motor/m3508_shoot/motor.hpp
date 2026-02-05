#pragma once

#include "motor/M3508.hpp"

inline PID::config_t speed_pid = {
    .kp = (20 * A) / (40 * rpm),
    .max_out = 20 * A,
};
inline M3508 motor({
    .can_port = 1,
    .master_id = 0x206,
    .slave_id = 0x200,
    .reduction = 268.0f / 17.0f,
    .Kt = M3508::Kt / M3508::REDUCTION * (268.0f / 17.0f),
    .is_invert = false,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &speed_pid,
});