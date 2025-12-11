#pragma once

#include "template/control.hpp"
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
        IMU::dir_t dir = {.yaw = 0 * deg, .pitch = 0 * deg, .roll = 0 * deg};
        IMU::calib_t calib = {.gx_offset = 0.00165860727, .gy_offset = -0.00199451833, .gz_offset = -0.00095358619, .g_norm = 9.78397942};
    } imu;

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
} config;