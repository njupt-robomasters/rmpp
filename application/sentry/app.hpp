#pragma once

#include "cmsis_os.h"

// bsp
#include "bsp.hpp"

// components
#include "unit.hpp"
#include "pid.hpp"
#include "imu.hpp"
#include "dj6.hpp"
#include "referee.hpp"

// module
#include "modules/chassis.hpp"
#include "modules/gimbal.hpp"
#include "modules/shooter.hpp"

class ModuleParams {
public:
    // IMU参数
    IMU::dir_t imu_dir = {.yaw = 90.0f, .pitch = 0, .roll = 0};
    IMU::calib_t imu_calib = {.gx_offset = 0.00130902638, .gy_offset = 0.00142620632, .gz_offset = 0.00215413119, .g_norm = 9.65372658};

    // 底盘PID参数
    PID::param_t servo_pid_param = {
        .kp = UnitFloat<A_deg>(0.6f * A_deg),
        .max_out = UnitFloat<A>(3.0f * A)
    };
    PID::param_t wheel_pid_param = {
        .kp = UnitFloat<A_rpm>(167.0f * A_rpm),
        .ki = UnitFloat<A_rpm>(0.83f * A_rpm),
        .max_out = UnitFloat<A>(20.0f * A)
    };

    // 云台PID参数
    PID::param_t yaw1_pid_param = {
        .kp = UnitFloat<A_deg>(0.35f * A_deg),
        .kd = UnitFloat<A_deg>(0.052f * A_deg),
        .max_out = UnitFloat<A>(10.0f * A)
    };
    PID::param_t yaw2_pid_param = {
        .kp = UnitFloat<A_deg>(1.0f * A_deg),
        .kd = UnitFloat<A_deg>(0.023f * A_deg),
        .max_out = UnitFloat<A>(3.0f * A)
    };
    PID::param_t pitch_pid_param = {
        .kp = UnitFloat<Nm_deg>(0.25f * Nm_deg),
        .kd = UnitFloat<Nm_deg>(0.012f * Nm_deg),
        .max_out = UnitFloat<Nm>(3.0f * Nm)
    };

    // 发射机构PID参数
    PID::param_t m2006_pid_param = {
        .kp = UnitFloat<A_rpm>(0.17f * A_rpm),
        .ki = UnitFloat<A_rpm>(0.67f * A_rpm),
        .max_i = UnitFloat<A>(5.0f * A),
        .max_out = UnitFloat<A>(10.0f * A)
    };
};

class AppParams {
public:
    // 底盘速度参数
    UnitFloat<m_s> vxy_max = 2.0f * m_s; // 前后左右平移速度
    UnitFloat<rpm> vr_max = 60.0f * rpm; // 旋转角速度
    UnitFloat<m_ss> axy = 2.0f * m_ss;

    // 云台速度参数
    UnitFloat<deg_s> yaw_max_speed = 240.0f * deg_s;
    UnitFloat<deg_s> pitch_max_speed = 240.0f * deg_s;

    // 键鼠参数
    float mouse_x_max = 100.0f;
    float mouse_y_max = 100.0f;
};

extern ModuleParams module_params;
extern AppParams app_params;
extern IMU imu;
extern DJ6 dj6;
extern Referee referee;
extern Chassis chassis;
extern Gimbal gimbal;
extern Shooter shooter;
