#pragma once

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/m3508.hpp"
#include "robot/chassis/chassis_matrix.hpp"

// 电机参数
static constexpr float REDUCTION = 268.0f / 17.0f;
static constexpr float KT = M3508::KT / M3508::REDUCTION * REDUCTION;
static constexpr bool IS_INVERT = false;

// 底盘参数
static constexpr float CHASSIS_RADIUS = 198.0f * mm;
static constexpr float WHEEL_RADIUS = 76.5f * mm;
static constexpr float MAX_F = M3508::MAX_CURRENT * KT / WHEEL_RADIUS * 4;

// 底盘PID参数
inline Pid::Config vxyz_pid = {
    .kp = MAX_F / (1),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
    .err_fc = 10,
};
inline Pid::Config follow_pid = {
    .kp = 5.0f,
    .max_out = 360 * deg,
};

// 底盘电机
inline M3508 w1({
    .bus = {.can = Can(1), .rx_id = 0x201},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});
inline M3508 w2({
    .bus = {.can = Can(1), .rx_id = 0x202},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});
inline M3508 w3({
    .bus = {.can = Can(1), .rx_id = 0x203},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});
inline M3508 w4({
    .bus = {.can = Can(1), .rx_id = 0x204},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});

// 底盘
inline ChassisMatrixTorque chassis(nullptr, {&w1, &w2, &w3, &w4}, {
                                                                      .kinematics = ChassisMatrix::OMNI,
                                                                      .chassis_radius = CHASSIS_RADIUS,
                                                                      .wheel_radius = WHEEL_RADIUS,
                                                                      .speed_pid_config = &vxyz_pid,
                                                                      .follow_pid_config = &follow_pid,
                                                                  });

void chassis_loop();
