#pragma once

#include "imu/IMU.hpp"

inline IMU::dir_t imu_dir = {.yaw = 0 * deg, .pitch = 0 * deg, .roll = 180 * deg};
inline IMU::calib_t imu_calib = {.gx_offset = 0.0019472912, .gy_offset = 0.000408173946, .gz_offset = 0.00394113874, .g_norm = 9.81952};
inline IMU imu(imu_dir, imu_calib);