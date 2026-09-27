#pragma once

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/dji/gm6020.hpp"
#include "motor/dji/m3508.hpp"
#include "robot/chassis/chassis_dual_steer.hpp"

// 电机参数
static constexpr float REDUCTION = 14.0f;
static constexpr float KT = M3508::DEFAULT_CONFIG.physical.kt / M3508::DEFAULT_CONFIG.install.reduction * REDUCTION;
static constexpr bool IS_WHEEL_INVERT = false;
static constexpr bool IS_SERVO_INVERT = true;

// 底盘参数
static constexpr float CHASSIS_RADIUS = 21.5f * cm;
static constexpr float WHEEL_RADIUS = 5.25f * cm;
static constexpr float MAX_F = M3508::DEFAULT_CONFIG.physical.max_current * KT / WHEEL_RADIUS * 2;

// 轮电机PID参数
inline Pid::Config wheel_pid = {
    .kp = (20) / (360 * rpm),
    .ki = (20) / (15 * deg),
    .max_out = 20,
};

// 舵电机PID参数
inline Pid::Config servo_pid = {
    .kp = (3) / (20 * deg),
    .max_out = 3,
};

// 底盘PID参数
inline Pid::Config vxyz_pid = {
    .kp = MAX_F / (1),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
};

// 轮电机
inline M3508 w1({
    .protocol = {.can = Can(1), .master_id = 0x201, .slave_id = 0x200},
    .physical = {.kt = KT},
    .install = {.is_invert = IS_WHEEL_INVERT, .reduction = REDUCTION},
    .control = {.control_mode = Motor::SPEED_MODE, .pid_output_type = Motor::CURRENT_OUTPUT, .speed_pid_config = &wheel_pid},
});
inline M3508 w2({
    .protocol = {.can = Can(1), .master_id = 0x202, .slave_id = 0x200},
    .physical = {.kt = KT},
    .install = {.is_invert = IS_WHEEL_INVERT, .reduction = REDUCTION},
    .control = {.control_mode = Motor::SPEED_MODE, .pid_output_type = Motor::CURRENT_OUTPUT, .speed_pid_config = &wheel_pid},
});

// 舵电机
inline GM6020 s1({
    .protocol = {.can = Can(1), .master_id = 0x207, .slave_id = 0x1FF},
    .install = {.offset = 135 * deg, .is_invert = IS_SERVO_INVERT},
    .control = {.control_mode = Motor::SINGLE_TURN_MODE, .pid_output_type = Motor::CURRENT_OUTPUT, .position_pid_config = &servo_pid},
});
inline GM6020 s2({
    .protocol = {.can = Can(1), .master_id = 0x208, .slave_id = 0x1FF},
    .install = {.offset = -15 * deg, .is_invert = IS_SERVO_INVERT},
    .control = {.control_mode = Motor::SINGLE_TURN_MODE, .pid_output_type = Motor::CURRENT_OUTPUT, .position_pid_config = &servo_pid},
});

// 底盘
inline ChassisDualSteer chassis({
                                    .chassis_radius = CHASSIS_RADIUS,
                                    .wheel_radius = WHEEL_RADIUS,
                                    .vxyz_pid_config = &vxyz_pid,
                                },
                                {w1, w2, s1, s2}, {.wheel_resistance = 0.194f / 2});

void chassis_loop();
