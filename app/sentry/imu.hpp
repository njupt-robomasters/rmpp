#pragma once

#include "bsp/bsp.hpp"

#include "imu/imu.hpp"
#include "math/unit.hpp"

inline Imu::Dir imu_dir = {.yaw = 90 * deg, .pitch = 0 * deg, .roll = 0 * deg};
inline Imu::Calib imu_calib = {.gx_offset = -0.000674135052, .gy_offset = 0.00137436262, .gz_offset = 0.00146490871, .g_norm = 9.54711819};
inline Imu imu(imu_dir, imu_calib, {.accel = bsp.spi(1, 1), .gyro = bsp.spi(1, 2), .heater = bsp.pwm(10, 1)});