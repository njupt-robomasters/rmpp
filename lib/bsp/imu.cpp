#include "bsp/imu.hpp"
#include <algorithm>
#include <cmath>

Imu::Imu(const Config& config)
    : config_(config), temperature_pid_(&config_.temperature_pid) {}

void Imu::start_calibration() {
    calibration_ = Calibration::STARTING;
}

void Imu::update_calibration(const Vector3& gyro, const Vector3& accel) {
    // 开始：延迟到更新周期开始采样，确保计时和样本计数同步
    if (calibration_ == Calibration::STARTING) {
        calibration_samples_ = 0;
        calibration_timer_.restart();
        calibration_ = Calibration::SAMPLING;
    }
    if (calibration_ != Calibration::SAMPLING)
        return;

    // 累计：更新陀螺仪三轴与加速度模长的极值和均值
    const float sample[4] = {gyro.x(), gyro.y(), gyro.z(), accel.norm()};
    if (calibration_samples_ == 0) {
        for (int i = 0; i < 4; i++)
            calibration_min_[i] = calibration_max_[i] = calibration_mean_[i] = sample[i];
    }
    calibration_samples_++;
    for (int i = 0; i < 4; i++) {
        calibration_min_[i] = std::min(calibration_min_[i], sample[i]);
        calibration_max_[i] = std::max(calibration_max_[i], sample[i]);
        calibration_mean_[i] += (sample[i] - calibration_mean_[i]) / calibration_samples_;
    }

    // 静止判断：加速度模长偏离重力，或波动超过陀螺仪 0.15 rad/s、加速度 0.5 m/s² 时提前判为失败
    bool disturbed = std::abs(sample[3] - 9.81f) > 0.5f;
    for (int i = 0; i < 4; i++) {
        const float range = i < 3 ? 0.15f : 0.5f;
        disturbed |= calibration_max_[i] - calibration_min_[i] > range;
    }
    if (disturbed) {
        calibration_ = Calibration::FAILED;
        return;
    }

    // 未满 5 s 时继续采样
    if (calibration_timer_.elapsed() < 5.0f)
        return;

    // 提交：平均零偏均不超过 0.01 rad/s 时写回配置，否则判为失败
    bool valid = true;
    for (int i = 0; i < 3; i++)
        valid &= std::abs(calibration_mean_[i]) <= 0.01f;
    if (valid) {
        config_.gyro_offset = {calibration_mean_[0], calibration_mean_[1], calibration_mean_[2]};
        config_.gravity = calibration_mean_[3];
        calibration_ = Calibration::SUCCEEDED;
    } else {
        calibration_ = Calibration::FAILED;
    }
}
