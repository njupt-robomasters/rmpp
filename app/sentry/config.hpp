#pragma once

#include "app/control.hpp"
#include "imu/imu.hpp"
#include "controller/pid.hpp"

inline struct {
    Control::speed_t speed = {
        // 底盘速度参数
        .vxy_max = 2.0f * m_s,
        .vr_max = 60.0f * rpm,
        .axy = 2.0f * m_ss,

        // 云台速度参数
        .yaw_max = 180.0f * deg_s,
        .pitch_max = 180.0f * deg_s,
    };

    // IMU参数
    struct {
        IMU::dir_t dir = {.yaw = 90 * deg, .pitch = 0 * deg, .roll = 0 * deg};
        IMU::calib_t calib = {.gx_offset = 0.00130902638, .gy_offset = 0.00142620632, .gz_offset = 0.00215413119, .g_norm = 9.65372658};
    } imu;

    // 底盘PID参数
    struct {
        PID::param_t servo_pid_param = {
            .mode = PID::POSITION_MODE,
            .kp = (3 * A) / (5.0f * deg),
            .max_out = 3 * A,
            .fc = 30 * Hz
        };
        PID::param_t wheel_pid_param = {
            .mode = PID::INCREMENT_MODE,
            .kp = (20 * A) / (120.0f * rpm),
            .ki = (20 * A) / (0.4f * rev),
            .max_out = 20 * A,
            .fc = 30 * Hz
        };
    } chassis;

    // 云台PID参数
    struct {
        PID::param_t yaw1_pid_param = {
            .mode = PID::POSITION_MODE,
            .kp = (10 * A) / (30.0f * A_deg),
            .kd = (10 * A) / (192.0f * deg_s),
            .max_out = 10 * A
        };
        PID::param_t yaw2_pid_param = {
            .mode = PID::POSITION_MODE,
            .kp = (3 * A) / (3.75f * deg),
            .kd = (3 * A) / (150.0f * deg_s),
            .max_out = 3 * A,
            .fc = 30 * Hz
        };
        PID::param_t pitch_pid_param = {
            .mode = PID::POSITION_MODE,
            .kp = (3 * Nm) / (12.0f * deg),
            .kd = (3 * Nm) / (250.0f * deg_s),
            .max_out = 3 * Nm
        };
    } gimbal;

    // 发射机构PID参数
    struct {
        PID::param_t shoot_pid_param = {
            .mode = PID::POSITION_MODE,
            .kp = (10 * A) / (20.0f * deg_s),
            .kd = (10 * A) / (2000.0f * deg_ss),
            .max_out = 10 * A,
            .fc = 30 * Hz
        };
    } shooter;
} config;