#pragma once

#include "motor/M3508.hpp"

// 底盘电机参数
static constexpr float REDUCTION = 14.0f;
static constexpr UnitFloat<Nm_A> Kt = M3508::Kt / M3508::REDUCTION * REDUCTION;
static constexpr bool IS_INVERT = false;

inline PID::config_t speed_pid = {
    .kp = (20 * A) / (120 * rpm),
    .ki = (20 * A) / (15 * deg),
    .max_i = 20 * A,
    .max_out = 20 * A,
};
inline M3508 motor({
    .can_port = 2,
    .master_id = 0x201,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &speed_pid,
});