#pragma once

#include "attitude/quaternion_ekf.hpp"
#include "motor/dm4310.hpp"
#include "motor/rmdl7025.hpp"
#include "robot/gimbal_fold.hpp"

// imu 倒装在 pitch2 头部
inline Imu imu({.orientation = euler_to_quaternion({.yaw = -90 * deg, .pitch = 0 * deg, .roll = 180 * deg}),
                .gyro_offset = Vector3f(0.00412f, 0.00166f, 0.00212f),
                .gravity = 9.80907f});
inline QuaternionEkf attitude(imu, {});

// 大 yaw：DM4310，CAN1
inline Pid::Config yaw1_speed_pid_config = {
    .kp = 5.0f,
    .ki = 0.3f,
    .max_i = 1.0f,
    .max_out = 7.0f,
};
inline Pid::Config yaw1_angle_pid_config = {
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
inline Pid::Config yaw2_speed_pid_config = {
    .kp = 5.0f,
    .ki = 0.3f,
    .max_i = 1.0f,
    .max_out = 8.5f,
};
inline Pid::Config yaw2_angle_pid_config = {
    .kp = 20.0f,
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
inline Pid::Config pitch1_speed_pid_config = {
    .max_out = 7.0f,
};
inline Pid::Config pitch1_angle_pid_config = {
    .kp = 0.0f,
    .max_out = 0.0f * deg_s,
};
inline DM4310 pitch1({
    .can = Can(2),
    .tx_id = 0x10,
    .rx_id = 0x20,
    .is_invert = false,
    .offset = 0 * deg,
});

// pitch2：俯仰轴，DM4310，CAN2
inline Pid::Config pitch2_speed_pid_config = {
    .kp = 0.50558f,
    .ki = 2.0f,
    .kd = 0.02f,
    .ff = -1.55f,
    .max_i = 0.4f,
    .max_out = 5.0f,
    .err_fc = 10,
};
inline Pid::Config pitch2_angle_pid_config = {
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
inline constexpr Angle PITCH_MIN = -8 * deg, PITCH_MAX = 18 * deg;

// 云台：yaw 使用世界系闭环与识别窗口，pitch2 保留编码器串级；限位使用关节编码器角度
inline GimbalFold gimbal({yaw1, yaw2, pitch1, pitch2}, {
                                                           .yaw1_angle_pid_config = &yaw1_angle_pid_config,
                                                           .yaw1_speed_pid_config = &yaw1_speed_pid_config,
                                                           .yaw2_angle_pid_config = &yaw2_angle_pid_config,
                                                           .yaw2_speed_pid_config = &yaw2_speed_pid_config,
                                                           .yaw2_min = -50 * deg,
                                                           .yaw2_max = 50 * deg,
                                                           .yaw_gyro_fc = 50.0f, // IMU yaw 速度一阶低通，Hz；0 直通
                                                           .pitch_angle_pid_config = &pitch2_angle_pid_config,
                                                           .pitch_speed_pid_config = &pitch2_speed_pid_config,
                                                           .pitch_min = PITCH_MIN,
                                                           .pitch_max = PITCH_MAX,
                                                           .pitch1_position_pid_config = &pitch1_angle_pid_config,
                                                           .pitch1_speed_pid_config = &pitch1_speed_pid_config,
                                                           .pitch1_min = -90 * deg, // TODO: 折叠极限
                                                           .pitch1_max = 0 * deg,   // TODO: 展开极限
                                                           .pitch1_gravity = {.torque = 0, .phase = 0 * deg},
                                                           .pitch2_gravity = {.torque = 0, .phase = 0 * deg},
                                                       });

void gimbal_loop();
