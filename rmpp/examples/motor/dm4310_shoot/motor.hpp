#pragma once

#include "motor/DM4310.hpp"

// 拨弹电机速度PID参数
inline PID::config_t speed_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (10 * Nm) / (200 * rpm),
    .ki = (10 * Nm) / (50 * deg),
    .max_i = 10 * Nm,
    .max_out = 10 * Nm,
};
// 拨弹电机
inline DM4310 motor({
    .can_port = 1,
    .master_id = 0x10,
    .slave_id = 0x11,
    .reduction = DM4310::reduction,
    .Kt = DM4310::Kt,
    .R = DM4310::R,
    .is_invert = true,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &speed_pid,
});