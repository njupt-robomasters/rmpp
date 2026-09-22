#pragma once

#include "bsp/bsp.hpp"

#include "imu/imu.hpp"
#include "math/unit.hpp"

inline Imu::Dir imu_dir = {.yaw = 0 * deg, .pitch = 0 * deg, .roll = 0 * deg};
inline Imu::Calib imu_calib = {.gx_offset = 0.00316276052, .gy_offset = -0.00411193585, .gz_offset = -0.00242328411, .g_norm = 9.79961872};
inline Imu imu(imu_dir, imu_calib, {.accel = bsp.spi(1, 1), .gyro = bsp.spi(1, 2), .heater = bsp.pwm(10, 1)});