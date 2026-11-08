#pragma once

#include "pid/pid.hpp"

#include "bsp/can.hpp"
#include "bsp/imu.hpp"

#include "attitude/quaternion_ekf.hpp"
#include "math/unit.hpp"
#include "motor/dm4310.hpp"
#include "motor/rmdl7025.hpp"
#include "robot/gimbal/gimbal_fold.hpp"

// 惯性测量单元与姿态解算：MC02 倒装在 pitch2 云台头上
// orientation：传感器到机体的安装旋转，Z-Y-X 欧拉角
// 旧框架实测为绕左右轴翻转 180°（旧 pitch = 180°）；旧安装矩阵为 Rz(-yaw) Rx(pitch) Ry(roll)，同一物理旋转即新 roll = 180°
// 新框架 attitude.pitch 绕机体 Y 轴（旧框架绕 X），与上游 hero 迁移一致未重排机体轴
// TODO: 上车确认枪管水平时 attitude.pitch / roll 接近 0°，抬枪口时 attitude.pitch 增大；不对时加 yaw = ±90° 使俯仰轴对齐机体 Y
// gyro_offset、gravity：用 jelly_sentry_debug-imu_calib 校准后填入；未校准时为 0 与 9.81
// temperature：调试阶段频繁 halt，沿用旧框架做法关闭加热（目标 0 ℃）；调试结束后改为 50，并在恒温下重新校准零偏
inline Imu imu({.orientation = euler_to_quaternion({.yaw = 0 * deg, .pitch = 0 * deg, .roll = 180 * deg}),
                .gyro_offset = Vector3f(0.0f, 0.0f, 0.0f),
                .gravity = 9.81f,
                .temperature = 0.0f});
inline QuaternionEkf attitude(imu, {});

// 达妙电机：tx_id 为调试助手中的 CAN ID，rx_id 为 Master ID；上位机 PMAX 改为 3.14159，MIT 模式
// 同一路总线上反馈按 data[0] 低 4 位区分电机，tx_id 低 4 位不能重复
// 以下 PID 均为旧框架初值原样迁移，新 PID 带积分回算、滤波改为误差输入低通，需上车复调

// 大 yaw：DM4310，CAN1，MIT 纯力矩；位置环 rad → rad/s，速度环 rad/s → Nm
inline Pid::Config yaw1_speed_pid = {
    .kp = (2.0f) / (100 * deg_s),
    .ki = (1.0f) / (100 * deg),
    .max_i = 1.0f,
    .max_out = 7.0f,
    .err_fc = 10,
};
inline Pid::Config yaw1_position_pid = {
    .kp = 5.0f,
    .max_out = 720 * deg_s,
};
inline DM4310 yaw1({
    .bus = {.can = Can(1), .tx_id = 0x01, .rx_id = 0x11},
    .drive = {.max_torque = DM4310::MAX_TORQUE, .kt = DM4310::KT, .reduction = DM4310::REDUCTION},
    .install = {.is_invert = false},
});

// 小 yaw：脉塔 RMD-L-7025，CAN2，电机 ID 1（控制帧 0x141，回复帧 0x241），电机只跑电流环
// 位置环 rad → rad/s，速度环 rad/s → A；偏心安装，重力与惯量前馈待评估
inline Pid::Config yaw2_speed_pid = {
    .kp = (1.0f) / (100 * deg_s),
    .ki = (0.5f) / (100 * deg),
    .max_i = 1.0f,
    .max_out = 6.0f,
    .err_fc = 10,
};
inline Pid::Config yaw2_position_pid = {
    .kp = 10.0f,
    .max_out = 720 * deg_s,
};
// TODO: 上车确认方向；offset 为小 yaw 回中时的原始角度，也可用上位机把当前位置写为电机零点
inline RMDL7025 yaw2({
    .bus = {.can = Can(2), .tx_id = 0x141, .rx_id = 0x241},
    .drive = {.max_current = RMDL7025::MAX_CURRENT, .max_torque = RMDL7025::MAX_TORQUE, .kt = RMDL7025::KT, .reduction = RMDL7025::REDUCTION},
    .install = {.is_invert = false, .offset = 0 * deg},
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
    .bus = {.can = Can(2), .tx_id = 0x11, .rx_id = 0x10},
    .drive = {.max_torque = DM4310::MAX_TORQUE, .kt = DM4310::KT, .reduction = DM4310::REDUCTION},
    .install = {.is_invert = false, .offset = 0 * deg},
});

// pitch2：俯仰轴，DM4310，CAN2，IMU 装在其上，走 ECD / IMU 模式
inline Pid::Config pitch2_speed_pid = {
    .kp = (1.2f) / (150 * deg_s),
    .ki = (0.3f) / (150 * deg),
    .kd = (0.5f) / (1000 * deg_ss),
    .max_i = 0.3f,
    .max_out = 4.0f,
    .err_fc = 10,
};
inline Pid::Config pitch2_position_pid = {
    .kp = 20.0f,
    .max_out = 720 * deg_s,
};
// offset 为枪管水平时的原始角度（实测）；TODO: 上车确认方向
inline DM4310 pitch2({
    .bus = {.can = Can(2), .tx_id = 0x13, .rx_id = 0x12},
    .drive = {.max_torque = DM4310::MAX_TORQUE, .kt = DM4310::KT, .reduction = DM4310::REDUCTION},
    .install = {.is_invert = false, .offset = -61.3f * deg},
});

// 云台：限位均为各自编码器角度；重力补偿力矩为 0 表示不补偿，折叠关节负载大，建议标定
inline GimbalFold gimbal(attitude, {yaw1, yaw2, pitch1, pitch2}, {
                                                                      .gimbal = {
                                                                          .yaw_speed_pid_config = &yaw1_speed_pid,
                                                                          .yaw_position_pid_config = &yaw1_position_pid,
                                                                          .pitch_speed_pid_config = &pitch2_speed_pid,
                                                                          .pitch_position_pid_config = &pitch2_position_pid,
                                                                          .yaw_output = Gimbal::TORQUE_OUTPUT,
                                                                          .pitch_output = Gimbal::TORQUE_OUTPUT,
                                                                          .pitch_min = -22 * deg,
                                                                          .pitch_max = 15 * deg,
                                                                          .is_pitch_limit = true,
                                                                      },
                                                                      .yaw2 = {
                                                                          .speed_pid_config = &yaw2_speed_pid,
                                                                          .position_pid_config = &yaw2_position_pid,
                                                                          .output = Gimbal::CURRENT_OUTPUT,
                                                                          .min = -40 * deg,
                                                                          .max = 40 * deg,
                                                                      },
                                                                      .pitch1 = {
                                                                          .speed_pid_config = &pitch1_speed_pid,
                                                                          .position_pid_config = &pitch1_position_pid,
                                                                          .output = Gimbal::TORQUE_OUTPUT,
                                                                          .min = -90 * deg, // TODO: 折叠极限
                                                                          .max = 0 * deg,   // TODO: 展开极限
                                                                      },
                                                                      .pitch1_gravity = {.torque = 0, .phase = 0 * deg},
                                                                      .pitch2_gravity = {.torque = 0, .phase = 0 * deg},
                                                                  });

void gimbal_loop();
