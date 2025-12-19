#pragma once

#include "lib.hpp"

inline struct {
    Robot::config_t config = {
        // 底盘速度
        .vxy_max = 3 * m_s,
        .wr_max = 60 * rpm,
        .axy = 4 * m_ss,
        .dxy = 4 * m_ss,

        // 云台速度
        .yaw_max = 360 * deg_s,
        .pitch_max = 360 * deg_s,

        // 发射机构
        .bullet_speed = 21 * m_s,
        .bullet_freq = 5 * Hz,
        .heat_protect = 20
    };

    // IMU参数
    struct {
        IMU::dir_t dir = {.yaw = 90 * deg, .pitch = 0 * deg, .roll = 0 * deg};
        IMU::calib_t calib = {.gx_offset = -0.000645550957, .gy_offset = 0.00187556574, .gz_offset = 0.0016447613, .g_norm = 9.57612038};
    } imu;

    // 底盘PID参数
    struct {
        PID::param_t wheel_pid = {
            .mode = PID::INCREMENT_MODE,
            .kp = (20 * A) / (120 * rpm),
            .ki = (20 * A) / (15 * deg),
            .max_out = 20 * A,
        };
        PID::param_t servo_pid = {
            .mode = PID::POSITION_MODE,
            .kp = (3 * A) / (10 * deg),
            .max_out = 3 * A,
        };
        PID::param_t follow_pid = {
            .mode = PID::POSITION_MODE,
            .kp = 1 / (0.2f * s),
            .max_out = 360 * deg_s
        };
    } chassis;

    // 云台PID参数
    struct {
        PID::param_t yaw1_pid = {
            .mode = PID::POSITION_MODE,
            .kp = (10 * A) / (10 * deg),
            .kd = (10 * A) / (100 * deg_s),
            .max_out = 10 * A,
            .fc = 10 * Hz
        };
        PID::param_t yaw2_pid = {
            .mode = PID::POSITION_MODE,
            .kp = (3 * A) / (20 * deg),
            .kd = (3 * A) / (300 * deg_s),
            .max_out = 3 * A,
            .fc = 30 * Hz
        };
        PID::param_t pitch_pid = {
            .mode = PID::POSITION_MODE,
            .kp = (3 * Nm) / (7.5f * deg),
            .kd = (3 * Nm) / (150 * deg_s),
            .max_out = 3 * Nm
        };
    } gimbal;

    // 发射机构PID参数
    struct {
        PID::param_t shoot_pid = {
            .mode = PID::POSITION_MODE,
            .kp = (10 * A) / (60 * deg_s),
            .max_out = 10 * A,
        };
    } shooter;
} config;