#include "bsp/imu.hpp"
#include <algorithm>
#include <cmath>

Imu::Imu(const Config& config)
    : orientation(config.orientation),
      target_temperature(config.temperature),
      temperature_pid_config(config.temperature_pid),
      gyro_offset(config.gyro_offset),
      gravity(config.gravity),
      temperature_pid(&temperature_pid_config) {}

void Imu::start_calibration() {
    calibration = Calibration::STARTING;
}

void Imu::update_calibration(const Vector3f& gyro, const Vector3f& accel) {
    // 开始：延迟到更新周期开始采样，确保计时和样本计数同步
    if (calibration == Calibration::STARTING) {
        calibration_samples = 0;
        calibration_timer.restart();
        calibration = Calibration::SAMPLING;
    }
    if (calibration != Calibration::SAMPLING)
        return;

    // 累计：更新陀螺仪三轴与加速度模长的极值和均值
    const float sample[4] = {gyro.x(), gyro.y(), gyro.z(), accel.norm()};
    if (calibration_samples == 0) {
        for (int i = 0; i < 4; i++)
            calibration_min[i] = calibration_max[i] = calibration_mean[i] = sample[i];
    }
    calibration_samples++;
    for (int i = 0; i < 4; i++) {
        calibration_min[i] = std::min(calibration_min[i], sample[i]);
        calibration_max[i] = std::max(calibration_max[i], sample[i]);
        calibration_mean[i] += (sample[i] - calibration_mean[i]) / calibration_samples;
    }

    // 静止判断：加速度模长偏离重力，或波动超过陀螺仪 0.15 rad/s、加速度 0.5 m/s² 时提前判为失败
    bool disturbed = std::abs(sample[3] - 9.81f) > 0.5f;
    for (int i = 0; i < 4; i++) {
        const float range = i < 3 ? 0.15f : 0.5f;
        disturbed |= calibration_max[i] - calibration_min[i] > range;
    }
    if (disturbed) {
        calibration = Calibration::FAILED;
        return;
    }

    // 未满 5 s 时继续采样
    if (calibration_timer.elapsed() < 5.0f)
        return;

    // 提交：平均零偏均不超过 0.01 rad/s 时写回标定字段，否则判为失败
    bool valid = true;
    for (int i = 0; i < 3; i++)
        valid &= std::abs(calibration_mean[i]) <= 0.01f;
    if (valid) {
        gyro_offset = {calibration_mean[0], calibration_mean[1], calibration_mean[2]};
        gravity = calibration_mean[3];
        calibration = Calibration::SUCCEEDED;
    } else {
        calibration = Calibration::FAILED;
    }
}
