#pragma once

// bsp
#include "bsp.hpp"

// components
#include "dj6.hpp"
#include "imu/imu.hpp"

// modules
#include "modules/chassis.hpp"
#include "modules/gimbal.hpp"
#include "modules/shooter.hpp"

struct Parameter {
    // IMU参数
    struct {
        IMU::dir_t dir = {.yaw = 90 * deg, .pitch = 0 * deg, .roll = 0 * deg};
        IMU::calib_t calib = {.gx_offset = 0.00130902638, .gy_offset = 0.00142620632, .gz_offset = 0.00215413119, .g_norm = 9.65372658};
    } imu;

    // 底盘PID参数
    struct {
        PID::param_t servo_pid_param = {
            .kp = 0.6f * A_deg,
            .max_out = 3.0f * A,
            .fc = 30.0f * Hz
        };
        PID::param_t wheel_pid_param = {
            .kp = 167.0f * A_rpm,
            .ki = 0.83f * A_rpm,
            .max_out = 20.0f * A,
            .fc = 30.0f * Hz
        };
    } chassis;

    // 云台PID参数
    struct {
        PID::param_t yaw1_pid_param = {
            .kp = 0.35f * A_deg,
            .kd = 0.052f * A_deg,
            .max_out = 10.0f * A
        };
        PID::param_t yaw2_pid_param = {
            .kp = 0.8f * A_deg,
            .kd = 0.02f * A_deg,
            .max_out = 3.0f * A,
            .fc = 30.0f * Hz
        };
        PID::param_t pitch_pid_param = {
            .kp = 0.25f * Nm_deg,
            .kd = 0.012f * Nm_deg,
            .max_out = 3.0f * Nm
        };
    } gimbal;

    // 发射机构PID参数
    struct {
        PID::param_t shoot_pid_param = {
            .kp = 0.5f * A_rpm,
            .kd = 0.005f * A_rpm,
            .max_out = 10.0f * A,
            .fc = 30.0f * Hz
        };
    } shooter;
};

struct Config {
    // 底盘速度参数
    UnitFloat<m_s> vxy_max = 2.0f * m_s; // 前后左右平移速度
    UnitFloat<rpm> vr_max = 60.0f * rpm; // 旋转角速度
    UnitFloat<m_ss> axy = 2.0f * m_ss;

    // 云台速度参数
    UnitFloat<deg_s> yaw_max_speed = 180.0f * deg_s;
    UnitFloat<deg_s> pitch_max_speed = 180.0f * deg_s;
};

struct Variable {
    struct {
        UnitFloat<m_s> rc, referee, nav, sum;
    } vx, vy, vr;

    struct {
        UnitFloat<deg_s> rc, referee, nav, sum;
    } yaw_speed, pitch_speed;
};

extern Parameter param;
extern Config cfg;
extern Variable var;

extern DJ6 dj6;
extern IMU imu;
extern Chassis chassis;
extern Gimbal gimbal;
extern Shooter shooter;
