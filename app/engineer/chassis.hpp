#pragma once

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/dji/m3508.hpp"
#include "robot/chassis/chassis_mecanum.hpp"

// 底盘电机参数
static constexpr float REDUCTION = 14.0f;
static constexpr float Kt = M3508::KT / M3508::REDUCTION * REDUCTION;
static constexpr bool IS_INVERT = true;

// 底盘参数
static constexpr float CHASSIS_RADIUS = 26.4f * cm;
static constexpr float WHEEL_RADIUS = 7.6f * cm;
static constexpr float MAX_F = M3508::MAX_CURRENT * Kt / WHEEL_RADIUS * 4;

// 底盘PID参数
inline Pid::Config vxyz_pid = {
    .kp = MAX_F / (1),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
    .fc = 10,
};
inline Pid::Config follow_pid = {};

// 底盘电机
inline M3508 w1({
    .bus = {.can = Can(2), .rx_id = 0x201},
    .drive = {.kt = Kt, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});
inline M3508 w2({
    .bus = {.can = Can(2), .rx_id = 0x202},
    .drive = {.kt = Kt, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});
inline M3508 w3({
    .bus = {.can = Can(2), .rx_id = 0x203},
    .drive = {.kt = Kt, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});
inline M3508 w4({
    .bus = {.can = Can(2), .rx_id = 0x204},
    .drive = {.kt = Kt, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});

// 底盘
inline ChassisMecanum chassis({
                                  .chassis_radius = CHASSIS_RADIUS,
                                  .wheel_radius = WHEEL_RADIUS,
                                  .vxyz_pid_config = &vxyz_pid,
                                  .follow_pid_config = &follow_pid,
                                  .steer_pid_config = &vxyz_pid,
                              },
                              {w1, w2, w3, w4});
