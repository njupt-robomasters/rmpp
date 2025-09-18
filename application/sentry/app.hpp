#pragma once

#include "cmsis_os.h"

// bsp
#include "bsp.hpp"

// components
#include "unit.hpp"
#include "pid.hpp"
#include "imu.hpp"
#include "dj6.hpp"

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
    PID::param_t servo_pid_param = {.kp = 0.6f * A / deg, .max_out = 3.0f * A};
    PID::param_t wheel_pid_param = {.kp = 10000.0f * A / rps, .ki = 50.0f * A / rps, .max_out = 20.0f * A};

    // 云台PID参数
    PID::param_t yaw1_pid_param = {.kp = 20.0f, .kd = 3.0f, .max_out = 10.0f};
    PID::param_t yaw2_pid_param = {.kp = 1.0f * A / deg, .kd = 0.023f * A / deg, .max_out = 3.0f * A};
    PID::param_t pitch_pid_param = {.kp = 0.25f * Nm / deg, .kd = 0.012f * Nm / deg, .max_out = 3.0f * Nm};

    // 发射机构PID参数
    PID::param_t m2006_pid_param = {.kp = 10.0f * A / rps, .ki = 40.0f * A / rps, .max_i = 5.0f, .max_out = 10.0f * A};
};

class AppParams {
public:
    // 底盘速度参数
    Unit<m_s> vxy_max = 2.0f * m_s; // 前后左右平移速度
    Unit<rpm> vr_max = 60.0f * rpm; // 旋转角速度

    // 云台速度参数
    Unit<deg_s> yaw_max_speed = 60.0f * deg_s;
    Unit<deg_s> pitch_max_speed = 60.0f * deg_s;
};

extern ModuleParams module_params;
extern AppParams app_params;
extern IMU imu;
extern DJ6 dj6;
extern Chassis chassis;
extern Gimbal gimbal;
extern Shooter shooter;
