#pragma once

#include "imu/IMU.hpp"

inline IMU::dir_t imu_dir = {.yaw = 0 * deg, .pitch = 0 * deg, .roll = 0 * deg};
inline IMU::calib_t imu_calib = {.gx_offset = 0.00316276052, .gy_offset = -0.00411193585, .gz_offset = -0.00242328411, .g_norm = 9.79961872};
inline IMU imu(imu_dir, imu_calib);