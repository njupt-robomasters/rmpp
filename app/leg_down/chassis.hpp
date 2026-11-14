#pragma once

#include "attitude/quaternion_ekf.hpp"
#include "lqr_fit.hpp"
#include "motor/dm8009.hpp"
#include "motor/m3508.hpp"
#include "power/super_cap.hpp"
#include "robot/chassis/chassis_leg.hpp"

// 底盘 C 板：传感器 +X 朝车后、+Y 朝车右、+Z 朝上，即相对机体系（X 前、Y 左、Z 上）绕 Z 转 180°
// 由外部工程腿倾角与横滚补偿的闭环极性反推；上车前用手低头应见 attitude.pitch 增大、右侧压低应见 attitude.roll 增大
inline Imu imu({.orientation = euler_to_quaternion({.yaw = 180 * deg, .pitch = 0 * deg, .roll = 0 * deg}),
                .gyro_offset = Vector3f(-0.00069f, -0.00059f, 0.00114f),
                .gravity = 9.5909f});
inline QuaternionEkf attitude(imu, {});

// 关节 DM8009：CAN2；面向输出轴逆时针为正，不反向
// offset 取外部工程 pos = wrap(raw + c) 中 c 的相反数，使 position 零位对应 A 杆方位角 π、E 杆 0
inline DM8009 left_front({
    .can = Can(2),
    .tx_id = 0x11,
    .rx_id = 0x21,
    .is_invert = false,
    .offset = -2.793f,
});
inline DM8009 left_back({
    .can = Can(2),
    .tx_id = 0x12,
    .rx_id = 0x22,
    .is_invert = false,
    .offset = -0.745f,
});
inline DM8009 right_back({
    .can = Can(2),
    .tx_id = 0x13,
    .rx_id = 0x23,
    .is_invert = false,
    .offset = -1.1478f,
});
inline DM8009 right_front({
    .can = Can(2),
    .tx_id = 0x14,
    .rx_id = 0x24,
    .is_invert = false,
    .offset = 1.225f,
});

// 轮 M3508：CAN2；左轮电机正转为后退，反向后转速与力矩均以前进为正
static constexpr float MAX_TORQUE = 5.0f;
static constexpr float REDUCTION = 16.875f;
inline M3508 left_wheel({
    .can = Can(2),
    .rx_id = 0x201,
    .max_torque = MAX_TORQUE,
    .reduction = REDUCTION,
    .is_invert = true,
});
inline M3508 right_wheel({
    .can = Can(2),
    .rx_id = 0x202,
    .max_torque = MAX_TORQUE,
    .reduction = REDUCTION,
    .is_invert = false,
});

// 腿长 PD：误差 m → 轴向力 N，伸腿为正
inline Pid::Config leg_length_pid = {.kp = 1750, .kd = 100, .max_out = 180};
// 相对腿角 PD：误差 rad → 摆动力矩 N·m，后摆为正
inline Pid::Config leg_angle_pid = {.kp = 140, .kd = 7, .max_out = 80};
// 横滚 PD：误差 rad → 腿长修正 m；微分用 gyro.x
inline Pid::Config roll_pid = {.kp = 1.3f, .kd = 0.004f, .max_out = 0.4f};

// 底盘：其余参数取 ChassisLeg::Config 默认值，与外部工程一致
inline ChassisLeg chassis({left_front, left_back, right_back, right_front, left_wheel, right_wheel},
                          {
                              .mass = 22.75f,
                              .spring_force = 450.0f,
                              .wheel_radius = 0.06f,
                              .wheel_distance = 0.488f,
                              .active_link = 0.210f,
                              .passive_link = 0.248f,
                              .leg_min = 0.14f,
                              .leg_max = 0.33f,
                              .leg_length_pid = &leg_length_pid,
                              .leg_angle_pid = &leg_angle_pid,
                              .roll_pid = &roll_pid,
                              .lqr_fit = LQR_FIT,
                          });

inline SuperCap super_cap({.can = Can(2)});

void chassis_loop();
