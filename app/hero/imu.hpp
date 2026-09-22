#pragma once

#include "bsp/bsp.hpp"

#include "imu/imu.hpp"
#include "math/unit.hpp"

inline Imu::Dir imu_dir = {.yaw = 0 * deg, .pitch = 0 * deg, .roll = 180 * deg};
inline Imu::Calib imu_calib = {.gx_offset = 0.0019472912, .gy_offset = 0.000408173946, .gz_offset = 0.00394113874, .g_norm = 9.81952};
inline Imu imu(imu_dir, imu_calib, {.accel = bsp.spi(1, 1), .gyro = bsp.spi(1, 2), .heater = bsp.pwm(10, 1)});