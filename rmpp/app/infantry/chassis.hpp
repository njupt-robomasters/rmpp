#pragma once

#include "motor/M3508.hpp"
#include "module/chassis/Chassis_Omni.hpp"

// 底盘电机参数
static constexpr float reduction = 268.0f / 17.0f;
static constexpr UnitFloat Kt = M3508::Kt / M3508::reduction * reduction;
static constexpr bool is_invert = false;

// 底盘电机PID参数
inline PID::config_t wheel_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (20 * A) / (120 * rpm),
    .ki = (20 * A) / (15 * deg),
    .max_i = 20 * A,
    .max_out = 20 * A
};

// 底盘电机
inline M3508 w1({
    .can_port = 1,
    .master_id = 0x201,
    .slave_id = 0x200,
    .reduction = reduction,
    .Kt = Kt,
    .R = M3508::R,
    .is_invert = is_invert,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid
});
inline M3508 w2({
    .can_port = 1,
    .master_id = 0x202,
    .slave_id = 0x200,
    .reduction = reduction,
    .Kt = Kt,
    .R = M3508::R,
    .is_invert = is_invert,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid
});
inline M3508 w3({
    .can_port = 1,
    .master_id = 0x203,
    .slave_id = 0x200,
    .reduction = reduction,
    .Kt = Kt,
    .R = M3508::R,
    .is_invert = is_invert,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid
});
inline M3508 w4({
    .can_port = 1,
    .master_id = 0x204,
    .slave_id = 0x200,
    .reduction = reduction,
    .Kt = Kt,
    .R = M3508::R,
    .is_invert = is_invert,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid
});

// 底盘
inline Chassis_Omni chassis({
                                .chassis_radius = 19.8f * cm,
                                .wheel_radius = 7.65f * cm
                            },
                            {w1, w2, w3, w4});