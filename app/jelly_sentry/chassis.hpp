#pragma once

#include "bsp/can.hpp"

#include "math/unit.hpp"
#include "motor/m3508.hpp"
#include "robot/chassis/chassis_matrix.hpp"

// 哨兵全向轮底盘：4×M3508 + C620，CAN1，电调 ID 左前 1、左后 2、右后 3、右前 4，控制帧 0x200
// 力控：底盘速度 PID 输出牵引力，力学逆解为各轮电流；轮子方向待用 jelly_sentry_debug-chassis 确认

// 电机参数：M3508 原装 3591/187 减速箱，KT 为输出轴力矩系数
static constexpr float REDUCTION = M3508::REDUCTION;
static constexpr float KT = M3508::KT / M3508::REDUCTION * REDUCTION;
static constexpr bool IS_WHEEL_INVERT = false;

// 底盘参数：按实车测量，底盘中心到轮 312.5 mm，轮半径 66 mm
static constexpr float CHASSIS_RADIUS = 312.5f * mm;
static constexpr float WHEEL_RADIUS = 66.0f * mm;
static constexpr float MAX_F = M3508::MAX_CURRENT * KT / WHEEL_RADIUS * 4; // 四轮满电流时的总牵引力，N

// 底盘速度 PID：速度误差 m/s → 牵引力 N
inline Pid::Config vxyz_pid = {
    .kp = MAX_F / (1),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
    .err_fc = 10,
};

// 底盘跟随 PID：云台相对底盘角度 rad → 旋转角速度 rad/s
inline Pid::Config follow_pid = {
    .kp = 5.0f,
    .max_out = 360 * deg,
};

// 底盘电机（左前、左后、右后、右前）
inline M3508 w1({
    .bus = {.can = Can(1), .rx_id = 0x201},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_CURRENT * KT, .kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_WHEEL_INVERT},
});
inline M3508 w2({
    .bus = {.can = Can(1), .rx_id = 0x202},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_CURRENT * KT, .kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_WHEEL_INVERT},
});
inline M3508 w3({
    .bus = {.can = Can(1), .rx_id = 0x203},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_CURRENT * KT, .kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_WHEEL_INVERT},
});
inline M3508 w4({
    .bus = {.can = Can(1), .rx_id = 0x204},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_CURRENT * KT, .kt = KT, .reduction = REDUCTION},
    .install = {.is_invert = IS_WHEEL_INVERT},
});

// 超级电容：未装，不调用 send()，按裁判缓冲能量做功率控制
inline SuperCap super_cap({.can = Can(1)});

// 底盘：轮电机线间电阻 0.194 Ω 用于功率估计
inline ChassisMatrixTorque chassis(super_cap, {&w1, &w2, &w3, &w4}, {
                                                                        .kinematics = ChassisMatrix::OMNI,
                                                                        .chassis_radius = CHASSIS_RADIUS,
                                                                        .wheel_radius = WHEEL_RADIUS,
                                                                        .wheel_resistance = 0.194f,
                                                                        .speed_pid_config = &vxyz_pid,
                                                                        .follow_pid_config = &follow_pid,
                                                                    });

void chassis_loop();
