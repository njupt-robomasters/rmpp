#pragma once

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/dji/m3508.hpp"
#include "robot/chassis/chassis_mecanum.hpp"

// 底盘电机参数
static constexpr float REDUCTION = 14.0f;
static constexpr float Kt = M3508::DEFAULT_CONFIG.physical.kt / M3508::DEFAULT_CONFIG.install.reduction * REDUCTION;
static constexpr bool IS_INVERT = true;

// 底盘参数
static constexpr float CHASSIS_RADIUS = 26.4f * cm;
static constexpr float WHEEL_RADIUS = 7.6f * cm;
static constexpr float MAX_F = M3508::DEFAULT_CONFIG.physical.max_current * Kt / WHEEL_RADIUS * 4;

// 底盘PID参数
inline Pid::Config vxyz_pid = {
    .kp = MAX_F / (1),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
    .fc = 10,
};

// 底盘电机
inline M3508 w1({
    .protocol = {.can = Can(2), .master_id = 0x201, .slave_id = 0x200},
    .physical = {.kt = Kt},
    .install = {.is_invert = IS_INVERT, .reduction = REDUCTION},
});
inline M3508 w2({
    .protocol = {.can = Can(2), .master_id = 0x202, .slave_id = 0x200},
    .physical = {.kt = Kt},
    .install = {.is_invert = IS_INVERT, .reduction = REDUCTION},
});
inline M3508 w3({
    .protocol = {.can = Can(2), .master_id = 0x203, .slave_id = 0x200},
    .physical = {.kt = Kt},
    .install = {.is_invert = IS_INVERT, .reduction = REDUCTION},
});
inline M3508 w4({
    .protocol = {.can = Can(2), .master_id = 0x204, .slave_id = 0x200},
    .physical = {.kt = Kt},
    .install = {.is_invert = IS_INVERT, .reduction = REDUCTION},
});

// 底盘
inline ChassisMecanum chassis({
                                  .chassis_radius = CHASSIS_RADIUS,
                                  .wheel_radius = WHEEL_RADIUS,
                                  .vxyz_pid_config = &vxyz_pid,
                              },
                              {w1, w2, w3, w4});
