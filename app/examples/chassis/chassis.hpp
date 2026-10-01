#pragma once

#include "bsp/can.hpp"

#include "motor/dji/c620.hpp"
#include "robot/chassis/chassis_omni.hpp"

// 电机参数
static constexpr float REDUCTION = 268.0f / 17.0f;
static constexpr float KT = C620::DRIVE.kt / C620::DRIVE.reduction * REDUCTION;
static constexpr bool IS_INVERT = false;

// 底盘参数
static constexpr float CHASSIS_RADIUS = 198.0f * mm;
static constexpr float WHEEL_RADIUS = 76.5f * mm;
static constexpr float MAX_F = C620::DRIVE.max_current * KT / WHEEL_RADIUS * 4;

// 底盘PID参数
inline Pid::Config vxyz_pid = {
    .kp = MAX_F / (1),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
    .fc = 10,
};
inline Pid::Config follow_pid = {
    .kp = 5.0f,
    .max_out = 360 * deg,
};

// 底盘电机
inline C620 w1({
    .bus = {.can = Can(1), .rx_id = 0x201},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});
inline C620 w2({
    .bus = {.can = Can(1), .rx_id = 0x202},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});
inline C620 w3({
    .bus = {.can = Can(1), .rx_id = 0x203},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});
inline C620 w4({
    .bus = {.can = Can(1), .rx_id = 0x204},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_INVERT},
});

// 底盘
inline ChassisOmni chassis({
                               .chassis_radius = CHASSIS_RADIUS,
                               .wheel_radius = WHEEL_RADIUS,
                               .vxyz_pid_config = &vxyz_pid,
                               .follow_pid_config = &follow_pid,
                           },
                           {w1, w2, w3, w4});
