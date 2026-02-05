#pragma once

#include "motor/M2006.hpp"

inline PID::config_t speed_pid = {
    .kp = (10 * A) / (20 * rpm),
    .max_out = 10 * A,
};
inline M2006 motor({
    .can_port = 1,
    .master_id = 0x207,
    .slave_id = 0x200,
    .is_invert = false,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &speed_pid,
});