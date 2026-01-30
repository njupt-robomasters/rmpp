#pragma once

#include "motor/M3508.hpp"

// 速度PID参数
inline PID::config_t speed_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (20 * A) / (40 * rpm),
    .max_out = 20 * A,
};

// 电机
inline M3508 motor({
    .can_port = 1,
    .master_id = 0x206,
    .slave_id = 0x200,
    .reduction = M3508::reduction,
    .Kt = M3508::Kt,
    .R = M3508::R,
    .is_invert = false,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &speed_pid,
});