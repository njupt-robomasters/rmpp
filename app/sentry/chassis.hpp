#pragma once

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/gm6020.hpp"
#include "motor/m3508.hpp"
#include "robot/chassis/chassis_steer.hpp"

// 电机参数
static constexpr float REDUCTION = 14.0f; // 轮电机总减速比暂用 14:1
static constexpr float KT = M3508::KT / M3508::REDUCTION * REDUCTION;
static constexpr bool IS_WHEEL_INVERT = false;
static constexpr bool IS_SERVO_INVERT = true;

// 底盘电机回传 125 Hz（8 ms），允许连续丢失约 5 帧
static constexpr float TIMEOUT = 40 * ms;

// 底盘参数：前后、左右舵轴间距均为 360 mm
static constexpr float CHASSIS_RADIUS = 180.0f * 1.41421356f * mm;
static constexpr float WHEEL_RADIUS = 60.0f * mm;

// 轮电机 PID：转速误差 rad/s → 电流 A
inline Pid::Config wheel_pid_config = {
    .kp = (20) / (360 * rpm),
    .ki = (20) / (15 * deg),
    .max_out = 20,
};

// 舵电机 PID：舵向误差 rad → 电流 A，电压帧 3 A ↔ 25 V，对应参考实现 7 V/40°、限幅 25 V
inline Pid::Config steer_pid = {
    .kp = (0.84f) / (40 * deg),
    .max_out = 3,
};

// 轮电机（左前、左后、右后、右前）
inline M3508 w1({
    .bus = {.can = Can(1), .rx_id = 0x201, .timeout = TIMEOUT},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_WHEEL_INVERT},
});
inline M3508 w2({
    .bus = {.can = Can(1), .rx_id = 0x202, .timeout = TIMEOUT},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_WHEEL_INVERT},
});
inline M3508 w3({
    .bus = {.can = Can(1), .rx_id = 0x203, .timeout = TIMEOUT},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_WHEEL_INVERT},
});
inline M3508 w4({
    .bus = {.can = Can(1), .rx_id = 0x204, .timeout = TIMEOUT},
    .drive = {.kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_WHEEL_INVERT},
});

// 舵电机（左前、左后、右后、右前），直驱；offset 为轮子朝前时的 raw 角度；电压帧反电动势前馈减半
inline GM6020 s1({
                     .bus = {.can = Can(1), .rx_id = 0x207, .timeout = TIMEOUT},
                     .drive = {.max_current = GM6020::MAX_CURRENT, .kt = GM6020::KT, .reduction = GM6020::REDUCTION},
                     .install = {.is_invert = IS_SERVO_INVERT, .offset = -133.5f * deg},
                 },
                 {.gain = 0.5f});
inline GM6020 s2({
                     .bus = {.can = Can(1), .rx_id = 0x208, .timeout = TIMEOUT},
                     .drive = {.max_current = GM6020::MAX_CURRENT, .kt = GM6020::KT, .reduction = GM6020::REDUCTION},
                     .install = {.is_invert = IS_SERVO_INVERT, .offset = 105.7f * deg},
                 },
                 {.gain = 0.5f});
inline GM6020 s3({
                     .bus = {.can = Can(1), .rx_id = 0x209, .timeout = TIMEOUT},
                     .drive = {.max_current = GM6020::MAX_CURRENT, .kt = GM6020::KT, .reduction = GM6020::REDUCTION},
                     .install = {.is_invert = IS_SERVO_INVERT, .offset = -168.5f * deg}, // 含 180° 修正轮子驱动方向
                 },
                 {.gain = 0.5f});
inline GM6020 s4({
                     .bus = {.can = Can(1), .rx_id = 0x20A, .timeout = TIMEOUT},
                     .drive = {.max_current = GM6020::MAX_CURRENT, .kt = GM6020::KT, .reduction = GM6020::REDUCTION},
                     .install = {.is_invert = IS_SERVO_INVERT, .offset = -18.4f * deg},
                 },
                 {.gain = 0.5f});

// 超级电容：未装，不调用 send()，离线时按估计功率控制
inline SuperCap super_cap({.can = Can(1)});

// 底盘；未配跟随 PID，跟随模式不旋转
inline ChassisSteerSpeed chassis(super_cap, {&w1, &w2, &w3, &w4}, {&s1, &s2, &s3, &s4}, {
                                                                                            .chassis_radius = CHASSIS_RADIUS,
                                                                                            .wheel_radius = WHEEL_RADIUS,
                                                                                            .wheel_resistance = 0.194f, // 不含舵电机
                                                                                            .speed_pid_config = &wheel_pid_config,
                                                                                            .steer_pid_config = &steer_pid,
                                                                                        });

void chassis_loop();
