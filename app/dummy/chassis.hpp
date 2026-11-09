#pragma once

#include "motor/m3508.hpp"
#include "robot/chassis/chassis_matrix.hpp"

// 轮速 PID，误差 rad/s → 电流 A；四轮共用
inline Pid::Config wheel_pid_config = {
    .kp = 1,
    .ki = 0,
    .max_i = 15.0f,
    .max_out = 15.0f,
};

inline Pid::Config follow_pid_config = {
    .kp = 1,
    .kd = 0,
    .max_out = 60 * rpm,
};

// 底盘电机：N630 电调已改为 C620 协议，按 M3508 驱动；左前、左后、右后、右前
inline M3508 w1({
    .bus = {.can = Can(1), .tx_id = 0x200, .rx_id = 0x201},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
    .install = {.is_invert = false},
});
inline M3508 w2({
    .bus = {.can = Can(1), .tx_id = 0x200, .rx_id = 0x202},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
    .install = {.is_invert = false},
});
inline M3508 w3({
    .bus = {.can = Can(1), .tx_id = 0x200, .rx_id = 0x203},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
    .install = {.is_invert = false},
});
inline M3508 w4({
    .bus = {.can = Can(1), .tx_id = 0x200, .rx_id = 0x204},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
    .install = {.is_invert = false},
});

// 超级电容：CAN1；靶车无裁判系统，电池侧功率上限与缓冲能量由 referee_loop 模拟
inline SuperCap super_cap({.can = Can(1)});

// 全向轮速度控制
inline ChassisMatrixSpeed chassis(super_cap, {&w1, &w2, &w3, &w4}, {
                                                                       .kinematics = ChassisMatrix::OMNI,
                                                                       .chassis_radius = 0.23f,
                                                                       .wheel_radius = 0.06f,
                                                                       .wheel_resistance = 0.194f,
                                                                       .speed_pid_config = &wheel_pid_config,
                                                                       .follow_pid_config = &follow_pid_config,
                                                                   });

void chassis_loop();
