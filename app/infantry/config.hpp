#pragma once

#include "lib.hpp"

inline struct {
    Robot::config_t config = {
        // 底盘速度
        .vxy_max = 2.0f * m_s,
        .vr_max = 60.0f * rpm,
        .axy = 4.0f * m_ss,
        .dxy = 8.0f * m_ss,

        // 云台速度
        .yaw_max = 360.0f * deg_s,
        .pitch_max = 360.0f * deg_s,

        // 发射机构
        .bullet_speed = 24.0f * m_s,
        .bullet_freq = 5.0f * Hz
    };

    // IMU参数
    struct {
        IMU::dir_t dir = {.yaw = 0 * deg, .pitch = 0 * deg, .roll = 0 * deg};
        IMU::calib_t calib = {.gx_offset = 0.00316276052, .gy_offset = -0.00411193585, .gz_offset = -0.00242328411, .g_norm = 9.79961872};
    } imu;

    // 底盘PID参数
    struct {
        PID::param_t wheel_pid = {
            .mode = PID::INCREMENT_MODE,
            .kp = (20 * A) / (120.0f * rpm),
            .ki = (20 * A) / (15.0f * deg),
            .max_out = 20 * A,
        };
    } chassis;

    // 云台PID参数
    struct {
        PID::param_t yaw_pid = {
            .mode = PID::POSITION_MODE,
            .kp = (3 * A) / (15.0f * deg),
            .kd = (3 * A) / (300.0f * deg_s),
            .max_out = 3 * A,
            .fc = 30 * Hz
        };
        PID::param_t pitch_pid = {
            .mode = PID::POSITION_MODE,
            .kp = (3 * Nm) / (15.0f * deg),
            .kd = (3 * Nm) / (200.0f * deg_s),
            .max_out = 3 * Nm
        };
    } gimbal;

    // 发射机构PID参数
    struct {
        PID::param_t shoot_pid = {
            .mode = PID::POSITION_MODE,
            .kp = (10 * A) / (20.0f * rpm),
            .max_out = 10 * A,
        };
    } shooter;
} config;