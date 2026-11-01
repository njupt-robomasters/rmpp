#pragma once

#include "pid/pid.hpp"

#include "bsp/can.hpp"
#include "bsp/imu.hpp"

#include "attitude/attitude.hpp"
#include "motor/dji/gm6020.hpp"
#include "motor/dm/dm4310.hpp"
#include "robot/gimbal/gimbal_classic.hpp"

// 惯性测量单元与姿态解算
inline Imu imu;
inline Attitude attitude(imu, {.gyro_offset = Vector3(0.00316276052f, -0.00411193585f, -0.00242328411f), .gravity = 9.79961872f});

// yaw电机
inline Pid::Config yaw_speed_pid = {
    .kp = (3) / (120 * deg_s),
    .max_out = 3,
    .fc = 10,
};
inline Pid::Config yaw_angle_pid = {
    .kp = 20.0f,
    .kd = 0.2,
    .max_out = 720 * deg_s,
};
inline GM6020 yaw({
    .bus = {.can = Can(1), .rx_id = 0x205},
    .drive = {.max_current = GM6020::MAX_CURRENT, .kt = GM6020::KT, .reduction = GM6020::REDUCTION},
    .install = {.is_invert = false},
});

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
    .install = {.offset = -150 * deg, .is_invert = false},
});

// 云台
inline GimbalClassic gimbal(attitude,
                            {.yaw_speed_pid_config = &yaw_speed_pid,
                             .yaw_position_pid_config = &yaw_angle_pid,
                             .pitch_speed_pid_config = &pitch_speed_pid,
                             .pitch_position_pid_config = &pitch_angle_pid,
                             .pitch_output = Gimbal::TORQUE_OUTPUT,
                             .pitch_min = -15 * deg,
                             .pitch_max = 30 * deg,
                             .is_pitch_limit = true},
                            {yaw, pitch});
