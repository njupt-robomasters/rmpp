#pragma once

#include "../leg_protocol.hpp"
#include "attitude/quaternion_ekf.hpp"
#include "motor/dm8009.hpp"
#include "motor/m3508.hpp"
#include "robot/chassis_leg_lqr.hpp"

// IMU：安装于车体，前进方向对齐机体 X 轴
inline Imu imu({.orientation = euler_to_quaternion({.yaw = 180 * deg, .pitch = 0 * deg, .roll = 0 * deg}),
                .gyro_offset = Vector3f(-0.00069f, -0.00059f, 0.00114f),
                .gravity = 9.5909f});
inline QuaternionEkf attitude(imu, {});

// 关节 DM8009：CAN1；面向输出轴逆时针为正，不反向
inline DM8009 left_front({
    .can = Can(1),
    .tx_id = 0x11,
    .rx_id = 0x21,
    .is_invert = false,
    .offset = -2.793f,
});
inline DM8009 left_back({
    .can = Can(1),
    .tx_id = 0x12,
    .rx_id = 0x22,
    .is_invert = false,
    .offset = -0.745f,
});
inline DM8009 right_back({
    .can = Can(1),
    .tx_id = 0x13,
    .rx_id = 0x23,
    .is_invert = false,
    .offset = -1.1478f,
});
inline DM8009 right_front({
    .can = Can(1),
    .tx_id = 0x14,
    .rx_id = 0x24,
    .is_invert = false,
    .offset = 1.225f,
});

// 轮 M3508：CAN1；左轮电机正转为后退，反向后转速与力矩均以前进为正
static constexpr float WHEEL_REDUCTION = 16.875f;
static constexpr float WHEEL_KT = 0.2545043441f; // 轮端等效转矩常数，N·m/A
inline M3508 left_wheel({
    .can = Can(1),
    .rx_id = 0x201,
    .max_torque = M3508::MAX_CURRENT * WHEEL_KT,
    .reduction = WHEEL_REDUCTION,
    .is_invert = true,
});
inline M3508 right_wheel({
    .can = Can(1),
    .rx_id = 0x202,
    .max_torque = M3508::MAX_CURRENT * WHEEL_KT,
    .reduction = WHEEL_REDUCTION,
    .is_invert = false,
});

// 底盘 PID 配置
inline Pid::Config leg_length_pid_config = {.kp = 1750, .kd = 100, .max_out = 350}; // 腿长 PD：m → 轴向力 N
inline Pid::Config leg_angle_pid_config = {.kp = 140, .kd = 7, .max_out = 80};      // 相对腿角 PD：rad → 摆动力矩 N·m
inline Pid::Config roll_pid_config = {.kp = 1.3f, .kd = 0.004f, .max_out = 0.4f};   // 横滚 PD：rad → 腿长修正 m

// 底盘：PID 参数在 app 层定义，LQR 系数来自 chassis_leg_lqr.hpp，其余参数取 ChassisLeg::Config 默认值
inline ChassisLeg chassis({left_front, left_back, right_back, right_front, left_wheel, right_wheel},
                          {
                              .leg_length_pid = &leg_length_pid_config,
                              .leg_angle_pid = &leg_angle_pid_config,
                              .roll_pid = &roll_pid_config,
                              .lqr_fit = LQR_FIT,
                              .lqr_posture_fit = LQR_POSTURE_FIT,
                              .lqr_r = LQR_R,
                          });

// 上下板 CAN1 经滑环连接，协议见 ../leg_protocol.hpp；接收回调在构造时注册
inline LegDownProtocol leg_protocol(Can(1));

// 底盘计算，每 2 次循环调用一次（500 Hz）
void chassis_loop();
