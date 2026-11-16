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
inline Pid::Config yaw_speed_pid_config = {
    .kp = 3.0f,
    .ki = 10.0f,
    .max_i = 1.0f,
    .max_out = GM6020::MAX_CURRENT,
};
inline Pid::Config yaw_angle_pid_config = {
    .kp = 0.0f,
    .kd = 0.0f,
    .max_out = 360 * deg_s,
};
inline GM6020 yaw({
    .can = Can(2),
    .tx_id = 0x1fe,
    .rx_id = 0x205,
    .is_invert = true,
    .offset = 5.048f,
});

// pitch电机
inline Pid::Config pitch_speed_pid_config = {
    .kp = 0.0f,
    .max_i = GM6020::MAX_CURRENT,
    .max_out = GM6020::MAX_CURRENT,
};
inline Pid::Config pitch_angle_pid_config = {
    .kp = 0.0f,
    .max_out = 360 * deg_s,
};
inline GM6020 pitch({
    .can = Can(2),
    .tx_id = 0x1ff,
    .rx_id = 0x206,
    .is_invert = true,
    .offset = 4.836f,
});

// 云台
inline GimbalClassic gimbal({yaw, pitch}, {
                                              .yaw_speed_pid_config = &yaw_speed_pid_config,
                                              .yaw_angle_pid_config = &yaw_angle_pid_config,
                                              .pitch_speed_pid_config = &pitch_speed_pid_config,
                                              .pitch_angle_pid_config = &pitch_angle_pid_config,
                                              .pitch_min = -30 * deg,
                                              .pitch_max = 15 * deg,
                                              .use_angle_loop = false,
                                          });

void gimbal_loop();
