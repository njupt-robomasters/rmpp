#pragma once

#include "attitude/quaternion_ekf.hpp"
#include "motor/dm4310.hpp"
#include "motor/rmdl7025.hpp"
#include "robot/gimbal_fold.hpp"

// IMU：MC02 倒装在 pitch2 云台头上；orientation 为传感器到机体的安装旋转（Z-Y-X 欧拉角）
// roll=180° 翻转左右轴，yaw=-90° 重排机体轴使俯仰轴对齐机体 Y；实测抬枪口时 attitude.pitch 与 pitch2.angle 同向
// gyro_offset、gravity 校准后填入；temperature 恒温 50 ℃，温度稳定后静止校准零偏
inline Imu imu({.orientation = euler_to_quaternion({.yaw = -90 * deg, .pitch = 0 * deg, .roll = 180 * deg}),
                .gyro_offset = Vector3f(0.0f, 0.0f, 0.0f),
                .gravity = 9.81f,
                .temperature = 50.0f});
inline QuaternionEkf attitude(imu, {});

// 达妙电机：tx_id 为调试助手中的 CAN ID，rx_id 为 Master ID；上位机 PMAX 改为 3.14159，MIT 模式
// 同一路总线上反馈按 data[0] 低 4 位区分电机，tx_id 低 4 位不能重复
// 以下 PID 为上车实测参数，带积分回算、误差输入低通

// 大 yaw：DM4310，CAN1，MIT 纯力矩；位置环 rad → rad/s，速度环 rad/s → Nm
inline Pid::Config yaw1_speed_pid = {
    .kp = 3.0f,
    .ki = 0.15f,
    .max_i = 1.0f,
    .max_out = 7.0f,
    .err_fc = 10,
};
inline Pid::Config yaw1_position_pid = {
    .kp = 5.0f,
    .max_out = 720 * deg_s,
};
inline DM4310 yaw1({
    .can = Can(1),
    .tx_id = 0x11,
    .rx_id = 0x10,
    .is_invert = true,
});

// 小 yaw：脉塔 RMD-L-7025，CAN3，电机 ID 1（控制帧 0x141，回复帧 0x241），电机只跑电流环
// 位置环 rad → rad/s，速度环 rad/s → A；偏心安装，重力与惯量前馈待评估
inline Pid::Config yaw2_speed_pid = {
    .kp = 2.0f,
    .ki = 0.1f,
    .max_i = 1.0f,
    .max_out = 8.5f,
    .err_fc = 10,
};
inline Pid::Config yaw2_position_pid = {
    .kp = 15.0f,
    .max_out = 720 * deg_s,
};
// 大小 yaw 正方向与 IMU 航向一致；offset 为小 yaw 回中时的原始角度
inline RMDL7025 yaw2({
    .can = Can(3),
    .tx_id = 0x141,
    .rx_id = 0x241,
    .is_invert = false,
    .offset = 3.192f,
});

// pitch1：折叠关节，DM4310，CAN2，只用编码器闭环；位置环限速 90°/s 限制折叠、展开速度
inline Pid::Config pitch1_speed_pid = {
    .kp = (1.2f) / (150 * deg_s),
    .ki = (0.3f) / (150 * deg),
    .kd = (0.5f) / (1000 * deg_ss),
    .max_i = 0.3f,
    .max_out = 4.0f,
    .err_fc = 10,
};
inline Pid::Config pitch1_position_pid = {
    .kp = 20.0f,
    .max_out = 90 * deg_s,
};
// TODO: 上车确认方向；offset 为展开（工作姿态）时的原始角度
inline DM4310 pitch1({
    .can = Can(2),
    .tx_id = 0x10,
    .rx_id = 0x20,
    .is_invert = false,
    .offset = 0 * deg,
});

// pitch2：俯仰轴，DM4310，CAN2，IMU 装在其上，走 ECD / IMU 模式
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
// offset 为枪管水平时的原始角度（实测）；TODO: 上车确认方向
inline DM4310 pitch2({
    .can = Can(2),
    .tx_id = 0x11,
    .rx_id = 0x21,
    .is_invert = false,
    .offset = -61.3f * deg,
});

// 云台：限位均为各自编码器角度；重力补偿力矩为 0 表示不补偿，折叠关节负载大，建议标定
inline GimbalFold gimbal({yaw1, yaw2, pitch1, pitch2}, {
                                                           .yaw_speed_pid_config = &yaw1_speed_pid,
                                                           .yaw_position_pid_config = &yaw1_position_pid,
                                                           .pitch_speed_pid_config = &pitch2_speed_pid,
                                                           .pitch_position_pid_config = &pitch2_position_pid,
                                                           .pitch_min = -22 * deg,
                                                           .pitch_max = 25 * deg,
                                                           .yaw2 = {
                                                               .speed_pid_config = &yaw2_speed_pid,
                                                               .position_pid_config = &yaw2_position_pid,
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
