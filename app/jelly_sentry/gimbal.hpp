#pragma once

#include "attitude/quaternion_ekf.hpp"
#include "motor/dm4310.hpp"
#include "motor/rmdl7025.hpp"
#include "robot/gimbal_fold.hpp"

// imu
inline Imu imu({.orientation = euler_to_quaternion({.yaw = -90 * deg, .pitch = 0 * deg, .roll = 180 * deg}),
                .gyro_offset = Vector3f(0.0f, 0.0f, 0.0f),
                .gravity = 9.81f});
inline QuaternionEkf attitude(imu, {});

// 大 yaw：DM4310，CAN1
inline Pid::Config yaw1_speed_pid = {
    .kp = 4.0f,
    .ki = 0.3f,
    .max_i = 1.0f,
    .max_out = 7.0f,
    .err_fc = 10,
};
inline Pid::Config yaw1_angle_pid = {
    .kp = 7.0f,
    .max_out = 720 * deg_s,
};
inline DM4310 yaw1({
    .can = Can(1),
    .tx_id = 0x11,
    .rx_id = 0x10,
    .is_invert = true,
});

// 小 yaw：脉塔 RMD-L-7025，CAN3
inline Pid::Config yaw2_speed_pid = {
    .kp = 2.0f,
    .ki = 0.1f,
    .max_i = 1.0f,
    .max_out = 8.5f,
    .err_fc = 10,
};
inline Pid::Config yaw2_angle_pid = {
    .kp = 15.0f,
    .max_out = 720 * deg_s,
};
inline RMDL7025 yaw2({
    .can = Can(3),
    .tx_id = 0x141,
    .rx_id = 0x241,
    .is_invert = false,
    .offset = 3.192f,
});

// pitch1：折叠关节，DM4310，CAN2
// 折叠云台库 1.2/(150 deg/s)、0.3/(150 deg)、0.5/(1000 deg/s^2) 展开为标准单位
inline Pid::Config pitch1_speed_pid = {
    .kp = 0.4584f,  // Nm/(rad/s)
    .ki = 0.1146f,  // Nm/rad
    .kd = 0.02865f, // Nm/(rad/s^2)
    .max_i = 0.3f,
    .max_out = 4.0f,
    .err_fc = 10,
};
inline Pid::Config pitch1_position_pid = {
    .kp = 20.0f,
    .max_out = 90 * deg_s,
};
inline DM4310 pitch1({
    .can = Can(2),
    .tx_id = 0x10,
    .rx_id = 0x20,
    .is_invert = false,
    .offset = 0 * deg,
});

// pitch2：俯仰轴，DM4310，CAN2
inline Pid::Config pitch2_speed_pid = {
    .kp = 0.50558f,
    .ki = 2.0f,
    .kd = 0.02f,
    .ff = -1.55f,
    .max_i = 0.4f,
    .max_out = 5.0f,
    .err_fc = 10,
};
inline Pid::Config pitch2_position_pid = {
    .kp = 20.0f,
    .max_out = 720 * deg_s,
};
inline DM4310 pitch2({
    .can = Can(2),
    .tx_id = 0x11,
    .rx_id = 0x21,
    .is_invert = false,
    .offset = -61.3f * deg,
});

// pitch2 编码器软限位与自主扫描端点，rad；下限为枪口最高点，上限为最低点
inline constexpr float PITCH_MIN = -0.1f, PITCH_MAX = 0.314f;

// 云台：限位均为各自编码器角度；重力补偿力矩为 0 表示不补偿，折叠关节负载大，建议标定
inline GimbalFold gimbal({yaw1, yaw2, pitch1, pitch2}, {
                                                           .yaw_speed_pid_config = &yaw1_speed_pid,
                                                           .yaw_position_pid_config = &yaw1_angle_pid,
                                                           .pitch_speed_pid_config = &pitch2_speed_pid,
                                                           .pitch_position_pid_config = &pitch2_position_pid,
                                                           .pitch_min = PITCH_MIN,
                                                           .pitch_max = PITCH_MAX,
                                                           .yaw2 = {
                                                               .speed_pid_config = &yaw2_speed_pid,
                                                               .position_pid_config = &yaw2_angle_pid,
                                                               .min = -40 * deg,
                                                               .max = 40 * deg,
                                                           },
                                                           .pitch1 = {
                                                               .speed_pid_config = &pitch1_speed_pid, .position_pid_config = &pitch1_position_pid,
                                                               .min = -90 * deg, // TODO: 折叠极限
                                                               .max = 0 * deg,   // TODO: 展开极限
                                                           },
                                                           .pitch1_gravity = {.torque = 0, .phase = 0 * deg},
                                                           .pitch2_gravity = {.torque = 0, .phase = 0 * deg},
                                                       });

void gimbal_loop();
