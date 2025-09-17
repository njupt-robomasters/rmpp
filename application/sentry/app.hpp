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
#include "module/chassis.hpp"
#include "module/shooter.hpp"

class Settings {
public:
    // IMU参数
    IMU::dir_t imu_dir = {.yaw = 90.0f, .pitch = 0, .roll = 0};
    IMU::calib_t imu_calib = {.gx_offset = 0.00130902638, .gy_offset = 0.00142620632, .gz_offset = 0.00215413119, .g_norm = 9.65372658};

    // 底盘PID参数
    PID::param_t m6020_pid_param = {.kp = 0.6f * A / deg, .max_out = 3.0f};
    PID::param_t m3508_pid_param = {.kp = 10000.0f * A / rps, .ki = 50.0f * A / rps, .max_out = 20.0f};

    // 发射机构PID参数
    PID::param_t m2006_pid_param = {.kp = 10.0f * A / rps, .ki = 40.0f * A / rps, .max_i = 5.0f, .max_out = 10.0f};

    // 底盘运动参数
    Unit<m_s> vxy_max = 3.0f * m_s; // 前后左右平移速度
    Unit<rpm> vr_max = 120.0f * rpm; // 旋转角速度
};

extern Settings settings;
extern IMU imu;
extern DJ6 dj6;
extern Chassis chassis;
extern Shooter shooter;
