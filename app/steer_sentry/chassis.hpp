#pragma once

#include "motor/gm6020.hpp"
#include "motor/m3508.hpp"
#include "power/super_cap.hpp"
#include "robot/chassis_steer.hpp"

// 电机参数：轮电机总减速比暂用 14:1
static constexpr float WHEEL_MAX_TORQUE = M3508::MAX_TORQUE / M3508::REDUCTION * 14.0f; // 按减速比换算的轮端满量程力矩

// 底盘参数：前后、左右舵轴间距均为 360 mm
static constexpr float CHASSIS_RADIUS = 0.25455844f; // 底盘中心到轮轴，m
static constexpr float WHEEL_RADIUS = 0.06f;         // 轮半径，m

// 导航满速平移时的小陀螺角速度，rad/s
static constexpr float WR_MIN = 60 * rpm;

// 导航目标点，上位机自定义坐标系；比赛去中心，测试去测试点，家为原点，m
static constexpr float GAME_CENTER_X = 2.2f, GAME_CENTER_Y = 4.7f;
static constexpr float TEST_CENTER_X = 1.0f, TEST_CENTER_Y = 0;
static constexpr float HOME_X = 0, HOME_Y = 0;

// 血量低于回家值时回家，回到中心值以上再去中心
static constexpr uint16_t GO_HOME_HP = 120, GO_CENTER_HP = 400;

// 轮电机 PID：转速误差 rad/s → 电流 A
inline Pid::Config wheel_pid_config = {
    .kp = 20.0f / (360.0f * rpm),
    .ki = 20.0f / (15.0f * deg),
    .max_out = 20.0f,
};

// 舵电机 PID：舵向误差 rad → 电流 A，电压帧 3 A ↔ 25 V，对应参考实现 7 V/40°、限幅 25 V
inline Pid::Config steer_pid = {
    .kp = 3.0f / (140 * deg),
    .max_out = 3.0f,
};

// 轮电机（左前、左后、右后、右前）
inline M3508 w1({
    .can = Can(1),
    .rx_id = 0x201,
    .max_torque = WHEEL_MAX_TORQUE,
    .reduction = 14.0f,
    .is_invert = false,
});
inline M3508 w2({
    .can = Can(1),
    .rx_id = 0x202,
    .max_torque = WHEEL_MAX_TORQUE,
    .reduction = 14.0f,
    .is_invert = false,
});
inline M3508 w3({
    .can = Can(1),
    .rx_id = 0x203,
    .max_torque = WHEEL_MAX_TORQUE,
    .reduction = 14.0f,
    .is_invert = false,
});
inline M3508 w4({
    .can = Can(1),
    .rx_id = 0x204,
    .max_torque = WHEEL_MAX_TORQUE,
    .reduction = 14.0f,
    .is_invert = false,
});

// 舵电机（左前、左后、右后、右前），直驱；offset 为轮子朝前时的 raw 角度；电压帧反电动势前馈取半
inline GM6020 s1({
                     .can = Can(1),
                     .rx_id = 0x207,
                     .is_invert = true,
                     .offset = -133.5f * deg,
                 },
                 0.5f * GM6020::KE);
inline GM6020 s2({
                     .can = Can(1),
                     .rx_id = 0x208,
                     .is_invert = true,
                     .offset = 105.7f * deg,
                 },
                 0.5f * GM6020::KE);
inline GM6020 s3({
                     .can = Can(1),
                     .rx_id = 0x209,
                     .is_invert = true,
                     .offset = -168.5f * deg,
                 },
                 0.5f * GM6020::KE);
inline GM6020 s4({
                     .can = Can(1),
                     .rx_id = 0x20A,
                     .is_invert = true,
                     .offset = -18.4f * deg,
                 },
                 0.5f * GM6020::KE);

// 超级电容：未装，不调用 send()，离线时按估计功率控制
inline SuperCap super_cap({.can = Can(1)});

// 底盘；未配跟随 PID，跟随模式不旋转
inline ChassisSteerSpeed chassis({&w1, &w2, &w3, &w4}, {&s1, &s2, &s3, &s4}, {
                                                                                 .chassis_radius = CHASSIS_RADIUS,
                                                                                 .wheel_radius = WHEEL_RADIUS,
                                                                                 .wheel_resistance = 0.194f, // 不含舵电机
                                                                                 .speed_pid_config = &wheel_pid_config,
                                                                                 .steer_pid_config = &steer_pid,
                                                                             });

void chassis_loop();
