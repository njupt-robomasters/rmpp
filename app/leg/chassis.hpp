#pragma once

#include "attitude/quaternion_ekf.hpp"
#include "bsp/can.hpp"
#include "bsp/imu.hpp"
#include "math/unit.hpp"
#include "motor/dm8009.hpp"
#include "motor/m3508.hpp"
#include "pid/pid.hpp"
#include "robot/chassis/chassis_leg.hpp"

// 参数来自 SOLDIER_LEG_4/Chassis（已实车验证）；极性核查见 agents/tasks/04-leg_chassis

// 底盘 C 板：传感器 +X 朝车后、+Y 朝车右、+Z 朝上，即相对机体系（X 前、Y 左、Z 上）绕 Z 转 180°
// 由外部工程腿倾角与横滚补偿的闭环极性反推；上车前用手低头应见 attitude.pitch 增大、右侧压低应见 attitude.roll 增大
// 零偏与重力模长在上电静止时校准，见 app_main
inline Imu imu({.orientation = euler_to_quaternion({.yaw = 180 * deg})});
inline QuaternionEkf attitude(imu);

// 关节 DM8009：CAN2；面向输出轴逆时针为正，不反向
// offset 取外部工程 pos = wrap(raw + c) 中 c 的相反数，使 position 零位对应 A 杆方位角 π、E 杆 0
inline DM8009 left_e({
    .bus = {.can = Can(2), .tx_id = 0x11, .rx_id = 0x21},
    .drive = {.max_torque = DM8009::MAX_TORQUE, .kt = DM8009::KT, .reduction = DM8009::REDUCTION},
    .install = {.offset = -2.754f, .is_invert = false},
});
inline DM8009 left_a({
    .bus = {.can = Can(2), .tx_id = 0x12, .rx_id = 0x22},
    .drive = {.max_torque = DM8009::MAX_TORQUE, .kt = DM8009::KT, .reduction = DM8009::REDUCTION},
    .install = {.offset = -0.798f, .is_invert = false},
});
inline DM8009 right_e({
    .bus = {.can = Can(2), .tx_id = 0x13, .rx_id = 0x23},
    .drive = {.max_torque = DM8009::MAX_TORQUE, .kt = DM8009::KT, .reduction = DM8009::REDUCTION},
    .install = {.offset = -1.115f, .is_invert = false},
});
inline DM8009 right_a({
    .bus = {.can = Can(2), .tx_id = 0x14, .rx_id = 0x24},
    .drive = {.max_torque = DM8009::MAX_TORQUE, .kt = DM8009::KT, .reduction = DM8009::REDUCTION},
    .install = {.offset = 1.259f, .is_invert = false},
});

// 轮 M3508：CAN2；左轮电机正转为后退，反向后转速与力矩均以前进为正
static constexpr float WHEEL_REDUCTION = 16.875f;
static constexpr float WHEEL_KT = 0.2545043441f; // 轮端等效转矩常数，N·m/A
inline M3508 left_wheel({
    .bus = {.can = Can(2), .rx_id = 0x201},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_CURRENT * WHEEL_KT, .kt = WHEEL_KT, .reduction = WHEEL_REDUCTION},
    .install = {.is_invert = true},
});
inline M3508 right_wheel({
    .bus = {.can = Can(2), .rx_id = 0x202},
    .drive = {.max_current = M3508::MAX_CURRENT, .max_torque = M3508::MAX_CURRENT * WHEEL_KT, .kt = WHEEL_KT, .reduction = WHEEL_REDUCTION},
    .install = {.is_invert = false},
});

// 腿长 PD：误差 m → 轴向力 N，伸腿为正
inline Pid::Config leg_length_pid = {.kp = 1750, .kd = 100, .max_out = 180};
// 相对腿角 PD：误差 rad → 摆动力矩 N·m，后摆为正
inline Pid::Config leg_angle_pid = {.kp = 140, .kd = 7, .max_out = 80};
// 横滚 PD：误差 rad → 腿长修正 m；微分用 gyro.x，外部工程为 10 阶最小二乘微分
inline Pid::Config roll_pid = {.kp = 1.3f, .kd = 0.004f, .max_out = 0.4f};

// LQR 增益拟合系数 [输出][状态][项]
// 输出：左轮、右轮（前进为正）、左摆、右摆（后摆为正）
// 状态：位移、速度、航向角、航向角速度、左腿倾角及角速度、右腿倾角及角速度、俯仰及角速度
// 项：1、左腿长、右腿长、左腿长²、左×右腿长、右腿长²
inline constexpr float LQR_FIT[4][10][6] = {
    {
        // 左轮力矩
        {-0.89054f, -6.588f, 5.6393f, 10.608f, -4.9395f, -4.991f},
        {-2.8378f, -14.357f, 16.696f, 27.54f, -19.53f, -13.877f},
        {-10.695f, 44.854f, -11.785f, -57.207f, 26.888f, 13.354f},
        {-1.9041f, 8.5527f, -3.4733f, -9.6422f, 5.8135f, 4.4717f},
        {-8.3131f, -91.763f, 9.1916f, 115.59f, -9.6928f, -11.063f},
        {-0.45493f, -3.372f, 1.2793f, -0.036797f, 0.75542f, -1.6115f},
        {-3.4168f, 7.444f, -5.7753f, -7.1263f, 17.538f, 3.0085f},
        {-0.23144f, -0.35352f, 0.029635f, 2.3872f, -4.9541f, 1.1267f},
        {-23.379f, 23.216f, 38.739f, 27.922f, -46.174f, -42.0f},
        {-3.3673f, 2.976f, 7.3557f, 4.4152f, -8.7858f, -7.9321f},
    },
    {
        // 右轮力矩
        {-0.89054f, 5.6393f, -6.588f, -4.991f, -4.9395f, 10.608f},
        {-2.8378f, 16.696f, -14.357f, -13.877f, -19.53f, 27.54f},
        {10.695f, 11.785f, -44.854f, -13.354f, -26.888f, 57.207f},
        {1.9041f, 3.4733f, -8.5527f, -4.4717f, -5.8135f, 9.6422f},
        {-3.4168f, -5.7753f, 7.444f, 3.0085f, 17.538f, -7.1263f},
        {-0.23144f, 0.029635f, -0.35352f, 1.1267f, -4.9541f, 2.3872f},
        {-8.3131f, 9.1916f, -91.763f, -11.063f, -9.6928f, 115.59f},
        {-0.45493f, 1.2793f, -3.372f, -1.6115f, 0.75542f, -0.036797f},
        {-23.379f, 38.739f, 23.216f, -42.0f, -46.174f, 27.922f},
        {-3.3673f, 7.3557f, 2.976f, -7.9321f, -8.7858f, 4.4152f},
    },
    {
        // 左摆动力矩
        {2.1481f, 4.7037f, -7.0789f, -23.157f, 15.608f, 5.1265f},
        {6.2928f, 7.2669f, -21.688f, -55.01f, 51.213f, 13.886f},
        {-7.2801f, -89.847f, -39.386f, 173.28f, -36.033f, 63.764f},
        {-1.0429f, -20.928f, -8.3873f, 38.244f, -12.097f, 13.658f},
        {58.602f, -67.864f, 9.7319f, 14.772f, 41.385f, -21.868f},
        {2.2102f, -1.9666f, -0.80115f, 0.6408f, 3.7021f, 0.099706f},
        {-1.4989f, -36.902f, -13.973f, 57.514f, -23.176f, -2.6731f},
        {-0.24454f, -0.40404f, 2.3883f, -3.1781f, 2.405f, -4.9149f},
        {-30.631f, -239.48f, 66.008f, 279.39f, 54.826f, -90.702f},
        {-3.0233f, -29.831f, 7.8705f, 30.573f, 11.901f, -11.342f},
    },
    {
        // 右摆动力矩
        {2.1481f, -7.0789f, 4.7037f, 5.1265f, 15.608f, -23.157f},
        {6.2928f, -21.688f, 7.2669f, 13.886f, 51.213f, -55.01f},
        {7.2801f, 39.386f, 89.847f, -63.764f, 36.033f, -173.28f},
        {1.0429f, 8.3873f, 20.928f, -13.658f, 12.097f, -38.244f},
        {-1.4989f, -13.973f, -36.902f, -2.6731f, -23.176f, 57.514f},
        {-0.24454f, 2.3883f, -0.40404f, -4.9149f, 2.405f, -3.1781f},
        {58.602f, 9.7319f, -67.864f, -21.868f, 41.385f, 14.772f},
        {2.2102f, -0.80115f, -1.9666f, 0.099706f, 3.7021f, 0.6408f},
        {-30.631f, 66.008f, -239.48f, -90.702f, 54.826f, 279.39f},
        {-3.0233f, 7.8705f, -29.831f, -11.342f, 11.901f, 30.573f},
    },
};

// 底盘：其余参数取 ChassisLeg::Config 默认值，与外部工程一致
inline ChassisLeg chassis({
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
                          },
                          {left_a, left_e, right_a, right_e, left_wheel, right_wheel});

void chassis_loop();
