#pragma once

#include "motor/M3508.hpp"
#include "motor/gm6020.hpp"
#include "module/chassis/Chassis_DualSteer.hpp"

// 底盘电机参数
static constexpr float reduction = 14.0f;
static constexpr UnitFloat Kt = M3508::Kt / M3508::reduction * reduction;
static constexpr bool is_wheel_invert = false;
static constexpr bool is_servo_invert = true;

// 底盘电机PID参数
inline PID::config_t wheel_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (20 * A) / (120 * rpm),
    .ki = (20 * A) / (15 * deg),
    .max_i = 20 * A,
    .max_out = 20 * A,
};
inline PID::config_t servo_pid = {
    .mode = PID::POSITION_MODE,
    .kp = (3 * A) / (20 * deg),
    .max_out = 3 * A,
};

// 底盘电机
inline M3508 w1({
    .can_port = 1,
    .master_id = 0x201,
    .slave_id = 0x200,
    .reduction = reduction,
    .Kt = Kt,
    .R = M3508::R,
    .is_invert = is_wheel_invert,
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
    .is_invert = is_wheel_invert,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid
});
inline GM6020 s1({
    .can_port = 1,
    .master_id = 0x207,
    .slave_id = 0x1FF,
    .reduction = GM6020::reduction,
    .Kt = GM6020::Kt,
    .R = GM6020::R,
    .is_invert = is_servo_invert,
    .offset = 135 * deg,
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &servo_pid
});
inline GM6020 s2({
    .can_port = 1,
    .master_id = 0x208,
    .slave_id = 0x1FF,
    .reduction = GM6020::reduction,
    .Kt = GM6020::Kt,
    .R = GM6020::R,
    .is_invert = is_servo_invert,
    .offset = -165 * deg,
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &servo_pid
});

// 底盘
inline Chassis_DualSteer chassis({
                                     .chassis_radius = 21.5f * cm,
                                     .wheel_radius = 5.25f * cm
                                 },
                                 {w1, w2, s1, s2});