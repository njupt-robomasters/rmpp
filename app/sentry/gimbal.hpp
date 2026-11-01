#pragma once

#include "pid/pid.hpp"

#include "bsp/can.hpp"
#include "bsp/imu.hpp"

#include "attitude/attitude.hpp"
#include "math/unit.hpp"
#include "motor/dm4310.hpp"
#include "motor/gm6020.hpp"
#include "robot/gimbal/gimbal_dual_yaw.hpp"

// 惯性测量单元与姿态解算
inline Imu imu;
inline Attitude attitude(imu, {.orientation = euler_to_quaternion({.yaw = -90 * deg}),
                               .gyro_offset = Vector3(-0.000674135052f, 0.00137436262f, 0.00146490871f),
                               .gravity = 9.54711819f});

// 用于Ozone调参
// #define yaw1_pid angle_pid
// #define yaw1 motor
//
// #define yaw2_speed_pid speed_pid
// #define yaw2_angle_pid angle_pid
// #define yaw2 motor
//
// #define pitch_speed_pid speed_pid
// #define pitch_angle_pid angle_pid
// #define pitch motor

// 大yaw电机：由 MF9025 换为 DM4310，CAN ID、PID 与安装方向待实车整定
// GimbalDualYaw 的 yaw_output 同时作用于大小 yaw，大 yaw 为 DM4310、小 yaw 为 GM6020，输出类型待调整
inline Pid::Config yaw1_pid = {
    .kp = (9) / (1.8 * deg),
    .kd = (9) / (18 * deg_s),
    .max_out = 9,
    .fc = 10,
};
inline DM4310 yaw1({
    .bus = {.can = Can(1), .tx_id = 0x12, .rx_id = 0x13},
    .drive = {.max_torque = DM4310::MAX_TORQUE, .kt = DM4310::KT, .reduction = DM4310::REDUCTION},
    .install = {.is_invert = false},
});

// 小yaw电机
inline Pid::Config yaw2_speed_pid = {
    .kp = (3) / (300 * deg_s),
    .kd = (3) / (30000 * deg_ss),
    .max_out = 3,
    .fc = 10,
};
inline Pid::Config yaw2_angle_pid = {
    .kp = 30.0f,
    .max_out = 720 * deg_s,
};
inline GM6020 yaw2({
                       .bus = {.can = Can(1), .rx_id = 0x205},
                       .drive = {.max_current = GM6020::MAX_CURRENT, .kt = GM6020::KT, .reduction = GM6020::REDUCTION},
                       .install = {.offset = 0 * deg},
                   },
                   {.gain = 1.0f}); // 电压帧完整反电动势前馈

// pitch电机
inline Pid::Config pitch_speed_pid = {
    .kp = (3) / (150 * deg_s),
    .max_out = 3,
    .fc = 10,
};
inline Pid::Config pitch_angle_pid = {
    .kp = 20.0f,
    .max_out = 720 * deg_s,
};
inline DM4310 pitch({
    .bus = {.can = Can(2), .tx_id = 0x11, .rx_id = 0x10},
    .drive = {.max_torque = DM4310::MAX_TORQUE, .kt = DM4310::KT, .reduction = DM4310::REDUCTION},
    .install = {.offset = -72 * deg, .is_invert = true},
});

// 云台
inline GimbalDualYaw gimbal(attitude,
                            {.yaw_speed_pid_config = &yaw2_speed_pid,
                             .yaw_position_pid_config = &yaw2_angle_pid,
                             .pitch_speed_pid_config = &pitch_speed_pid,
                             .pitch_position_pid_config = &pitch_angle_pid,
                             .pitch_output = Gimbal::TORQUE_OUTPUT,
                             .pitch_min = -22 * deg,
                             .pitch_max = 15 * deg,
                             .is_pitch_limit = true},
                            {yaw1, yaw2, pitch});

void gimbal_loop();
