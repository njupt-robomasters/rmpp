#pragma once

#include "math/Conversions.hpp"
#include "module/chassis/Chassis_DualSteer.hpp"
#include "motor/GM6020.hpp"
#include "motor/M3508.hpp"

using math::cm;
using math::deg;
using math::rpm;

// 电机参数
static constexpr float REDUCTION = 14.0f;
static constexpr float Kt = M3508::Kt / M3508::REDUCTION * REDUCTION;
static constexpr bool IS_WHEEL_INVERT = false;
static constexpr bool IS_SERVO_INVERT = true;

// 底盘参数
static constexpr float CHASSIS_RADIUS = 21.5f * cm;
static constexpr float WHEEL_RADIUS = 5.25f * cm;
static constexpr float MAX_F = M3508::MAX_CURRENT * Kt / WHEEL_RADIUS * 2;

// 轮电机PID参数
inline PID::config_t wheel_pid = {
    .kp = (20) / (360 * rpm),
    .ki = (20) / (15 * deg),
    .max_out = 20,
};

// 舵电机PID参数
inline PID::config_t servo_pid = {
    .kp = (3) / (20 * deg),
    .max_out = 3,
};

// 底盘PID参数
inline PID::config_t vxyz_pid = {
    .kp = MAX_F / (1),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
};

// 轮电机
inline M3508 w1({
    .can_port = 1,
    .master_id = 0x201,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_WHEEL_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid,
});
inline M3508 w2({
    .can_port = 1,
    .master_id = 0x202,
    .slave_id = 0x200,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_WHEEL_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid,
});

// 舵电机
inline GM6020 s1({
    .can_port = 1,
    .master_id = 0x207,
    .slave_id = 0x1FF,
    .is_invert = IS_SERVO_INVERT,
    .offset = 135 * deg,
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &servo_pid,
});
inline GM6020 s2({
    .can_port = 1,
    .master_id = 0x208,
    .slave_id = 0x1FF,
    .is_invert = IS_SERVO_INVERT,
    .offset = -15 * deg,
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &servo_pid,
});

// 底盘
inline Chassis_DualSteer chassis({
                                     .chassis_radius = CHASSIS_RADIUS,
                                     .wheel_radius = WHEEL_RADIUS,
                                     .vxyz_pid_config = &vxyz_pid,
                                 },
                                 {w1, w2, s1, s2});