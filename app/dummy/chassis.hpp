#pragma once

#include "motor/m3508.hpp"
#include "motor/x630.hpp"
#include "robot/chassis/chassis_matrix.hpp"

// 轮速 PID，误差 rad/s → 电流 A；四轮共用
inline Pid::Config wheel_pid_config = {
    .kp = 1,
    .ki = 0,
    .max_i = 20.0f,
    .max_out = 20.0f,
};

inline Pid::Config follow_pid_config = {
    .kp = 10.0f,
    .kd = 1.0f,
    .max_out = 60 * rpm,
};

// 底盘电机：N630，C620 协议；HFI 静音用默认滞回阈值；左前、左后、右后、右前
inline X630 w1({
                   .bus = {.can = Can(1), .tx_id = 0x200, .rx_id = 0x201},
                   .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
                   .install = {.is_invert = false},
               },
               {});
inline X630 w2({
                   .bus = {.can = Can(1), .tx_id = 0x200, .rx_id = 0x202},
                   .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
                   .install = {.is_invert = false},
               },
               {});
inline X630 w3({
                   .bus = {.can = Can(1), .tx_id = 0x200, .rx_id = 0x203},
                   .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
                   .install = {.is_invert = false},
               },
               {});
inline X630 w4({
                   .bus = {.can = Can(1), .tx_id = 0x200, .rx_id = 0x204},
                   .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_TORQUE, .kt = M3508::KT, .reduction = M3508::REDUCTION},
                   .install = {.is_invert = false},
               },
               {});

// 超级电容：CAN1；靶车无裁判系统，电池侧功率上限与缓冲能量由 referee_loop 模拟
inline SuperCap super_cap({.can = Can(1)});

// 全向轮速度控制
inline ChassisMatrixSpeed chassis({&w1, &w2, &w3, &w4}, super_cap,
                                  {
                                      .kinematics = ChassisMatrix::OMNI,
                                      .chassis_radius = 0.23f,
                                      .wheel_radius = 0.06f,
                                      .wheel_resistance = 0.194f,
                                      .speed_pid_config = &wheel_pid_config,
                                      .follow_pid_config = &follow_pid_config,
                                  });

void chassis_loop();
