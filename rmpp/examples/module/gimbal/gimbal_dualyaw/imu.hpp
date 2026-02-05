#pragma once

#include "imu/IMU.hpp"

inline IMU::dir_t imu_dir = {.yaw = 90 * deg, .pitch = 0 * deg, .roll = 0 * deg};
inline IMU::calib_t imu_calib = {.gx_offset = -0.000674135052, .gy_offset = 0.00137436262, .gz_offset = 0.00146490871, .g_norm = 9.54711819};
inline IMU imu(imu_dir, imu_calib);