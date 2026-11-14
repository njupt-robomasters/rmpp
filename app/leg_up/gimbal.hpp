#pragma once

#include "attitude/quaternion_ekf.hpp"
#include "motor/gm6020.hpp"
#include "robot/gimbal/gimbal_classic.hpp"

// 惯性测量单元与姿态解算
inline Imu imu({.orientation = euler_to_quaternion({.yaw = 180 * deg, .pitch = 0 * deg, .roll = 0 * deg}),
                .gyro_offset = Vector3f(0.00127f, 0.00077f, 0.000271f),
                .gravity = 9.7423f});
inline QuaternionEkf attitude(imu, {});

// yaw电机
inline Pid::Config yaw_speed_pid = {
    .kp = GM6020::MAX_CURRENT / (120 * deg_s),
    .max_out = 3,
    .err_fc = 10,
};
inline Pid::Config yaw_position_pid = {
    .kp = 10.0f,
    .kd = 0.0f,
    .max_out = 720 * deg_s,
};
static GM6020 yaw({
    .can = Can(2),
    .tx_id = 0x1FF,
    .rx_id = 0x205,
    .is_invert = false,
});

// pitch电机
inline Pid::Config pitch_speed_pid = {
    .kp = 3 / (150 * deg_s),
    .max_out = 3,
    .err_fc = 10,
};
inline Pid::Config pitch_position_pid = {
    .kp = 10.0f,
    .max_out = 720 * deg_s,
};
static GM6020 pitch({
    .can = Can(2),
    .tx_id = 0x1FF,
    .rx_id = 0x206,
    .is_invert = false,
});

// 云台
inline GimbalClassic gimbal({yaw, pitch}, {
                                              .yaw_speed_pid_config = &yaw_speed_pid,
                                              .yaw_position_pid_config = &yaw_position_pid,
                                              .yaw_output = Gimbal::CURRENT_OUTPUT,
                                              .pitch_speed_pid_config = &pitch_speed_pid,
                                              .pitch_position_pid_config = &pitch_position_pid,
                                              .pitch_output = Gimbal::TORQUE_OUTPUT,
                                              .pitch_min = -15 * deg,
                                              .pitch_max = 30 * deg,
                                          });

void gimbal_loop();
