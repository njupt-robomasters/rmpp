#pragma once

#include "motor/M3508.hpp"
#include "module/chassis/Chassis_Mecanum.hpp"

static constexpr float reduction = 268.0f / 17.0f;
static constexpr bool is_invert = false;
static constexpr UnitFloat Kt = 0.2187f * Nm_A;

// 电机
inline PID::config_t wheel_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (20 * A) / (60 * rpm),
    .ki = (20 * A) / (10 * deg),
    .max_i = 20 * A,
    .max_out = 20 * A
};
inline Motor::config_t w1_config = {
    .can_port = 1,
    .master_id = 0x201,
    .slave_id = 0x200,
    .reduction = reduction,
    .Kt = Kt,
    .R = M3508::R,
    .is_invert = is_invert,
    .control_mode = Motor::SPEED_MODE,
    .speed_pid_output = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid
};
inline M3508 w1(w1_config);
inline Motor::config_t w2_config = {
    .can_port = 1,
    .master_id = 0x202,
    .slave_id = 0x200,
    .reduction = reduction,
    .Kt = Kt,
    .R = M3508::R,
    .is_invert = is_invert,
    .control_mode = Motor::SPEED_MODE,
    .speed_pid_output = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid
};
inline M3508 w2(w2_config);
inline Motor::config_t w3_config = {
    .can_port = 1,
    .master_id = 0x202,
    .slave_id = 0x200,
    .reduction = reduction,
    .Kt = Kt,
    .R = M3508::R,
    .is_invert = is_invert,
    .control_mode = Motor::SPEED_MODE,
    .speed_pid_output = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid
};
inline M3508 w3(w3_config);
inline Motor::config_t w4_config = {
    .can_port = 1,
    .master_id = 0x202,
    .slave_id = 0x200,
    .reduction = reduction,
    .Kt = Kt,
    .R = M3508::R,
    .is_invert = is_invert,
    .control_mode = Motor::SPEED_MODE,
    .speed_pid_output = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid
};
inline M3508 w4(w4_config);

// 底盘
inline Chassis::config_t chassis_config = {
    .chassis_radius = 281.96f * cm,
    .wheel_radius = 10.5f * cm
};
inline Chassis_Mecanum::motor_t chassis_motor = {w1, w2, w3, w4};
inline Chassis_Mecanum chassis(chassis_config, chassis_motor);