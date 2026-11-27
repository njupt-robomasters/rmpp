#pragma once

#include "attitude/quaternion_ekf.hpp"
#include "motor/dm4310.hpp"
#include "motor/gm6020.hpp"
#include "robot/gimbal_dual_yaw.hpp"

// 惯性测量单元与姿态解算
inline Imu imu({.orientation = euler_to_quaternion({.yaw = -90 * deg}),
                .gyro_offset = Vector3f(-0.000674135052f, 0.00137436262f, 0.00146490871f),
                .gravity = 9.54711819f});
inline QuaternionEkf attitude(imu, {});

// 大yaw电机：由 MF9025 换为 DM4310，CAN ID 与安装方向待实车整定
// 大小 yaw 共用下方小 yaw PID 输出：大 yaw 按 N·m、小 yaw 按 A 解释，参数待实车整定
inline DM4310 yaw1({
    .can = Can(1),
    .tx_id = 0x12,
    .rx_id = 0x13,
    .is_invert = false,
});

// 小yaw电机
inline Pid::Config yaw2_speed_pid_config = {
    .kp = (3) / (300 * deg_s),
    .kd = (3) / (30000 * deg_ss),
    .max_out = 3,
    .err_fc = 10,
};
inline Pid::Config yaw2_angle_pid_config = {
    .kp = 30.0f,
    .max_out = 720 * deg_s,
};
inline GM6020 yaw2({
                       .can = Can(1),
                       .rx_id = 0x205,
                       .offset = 0 * deg,
                   },
                   GM6020::KE); // 电压帧完整反电动势前馈

// pitch电机
inline Pid::Config pitch_speed_pid_config = {
    .kp = (3) / (150 * deg_s),
    .max_out = 3,
    .err_fc = 10,
};
inline Pid::Config pitch_angle_pid_config = {
    .kp = 20.0f,
    .max_out = 720 * deg_s,
};
inline DM4310 pitch({
    .can = Can(2),
    .tx_id = 0x11,
    .rx_id = 0x10,
    .is_invert = true,
    .offset = -72 * deg,
});

// 云台
inline GimbalDualYaw gimbal({yaw1, yaw2, pitch}, {
                                                     .yaw_speed_pid_config = &yaw2_speed_pid_config,
                                                     .yaw_angle_pid_config = &yaw2_angle_pid_config,
                                                     .pitch_speed_pid_config = &pitch_speed_pid_config,
                                                     .pitch_angle_pid_config = &pitch_angle_pid_config,
                                                     .pitch_min = -22 * deg,
                                                     .pitch_max = 15 * deg,
                                                 });

// 自主扫描：yaw 匀速旋转，pitch 在 IMU 范围内往返，rad/s
static constexpr float YAW_SPEED = 180 * deg_s, PITCH_SPEED = 60 * deg_s;
static constexpr Angle PITCH_MIN = -22 * deg, PITCH_MAX = 0 * deg;

// 自主锁定：自瞄有效距离，m；脱锁时间，s
static constexpr float LOCK_DISTANCE = 5, LOCK_TIMEOUT = 500 * ms;

void gimbal_loop(bool is_detect, Angle target_yaw, Angle target_pitch, float distance);
