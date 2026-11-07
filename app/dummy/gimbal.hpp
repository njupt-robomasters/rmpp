#pragma once

#include "attitude/quaternion_ekf.hpp"
#include "motor/rmdl7025.hpp"

// 惯性测量单元与姿态解算，C 板装在底盘上，结合 yaw 电机角度换算云台朝向；静止漂移明显时重新校准
inline Imu imu({.gyro_offset = Vector3f(0.00159558014f, -0.00282915123f, -0.000537958229f), .gravity = 9.70215511f});
inline QuaternionEkf attitude(imu, {});

// 云台位置 P 控制，世界系朝向误差 rad → 电流 A；待实车整定
inline Pid::Config yaw_pid_config = {
    .kp = RMDL7025::MAX_CURRENT / (85.0f * deg),
    .max_out = RMDL7025::MAX_CURRENT,
};

// 云台电机：RMD-L-7025 直连 CAN1，电机 ID 1；position 为云台相对底盘前方的角度，逆时针为正，零点待标定
inline RMDL7025 yaw({
    .bus = {.can = Can(1), .tx_id = 0x141, .rx_id = 0x241},
    .drive = {.max_current = RMDL7025::MAX_CURRENT, .max_torque = RMDL7025::MAX_TORQUE, .kt = RMDL7025::KT, .reduction = RMDL7025::REDUCTION},
    .install = {.is_invert = false, .offset = 0},
});

void gimbal_loop();
