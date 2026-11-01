#include "attitude.hpp"
#include <algorithm>
#include <cmath>

Attitude::Attitude(Imu& imu, const Config& config)
    : imu_(imu), config_(config), ekf_(config.ekf) {}

void Attitude::update() {
    if (!imu_.update())
        return;

    update_calibration();

    // 零偏、重力模长与安装方向修正
    gyro = config_.orientation * (imu_.gyro - config_.gyro_offset);
    accel = config_.orientation * (imu_.accel * (9.81f / config_.gravity));

    // 姿态解算
    ekf_.update(gyro, accel);
    q = ekf_.q;

    // 四元数 -> 欧拉角
    const Angle last_yaw = yaw;
    const auto euler = quaternion_to_euler(q);
    yaw = euler.yaw;
    pitch = euler.pitch;
    roll = euler.roll;
    yaw_total_angle += Angle(yaw - last_yaw);
}

void Attitude::start_calibration() {
    calibration_ = Calibration::STARTING;
}

void Attitude::update_calibration() {
    // 延迟到更新周期开始新窗口，确保计时和样本计数同步
    if (calibration_ == Calibration::STARTING) {
        calibration_samples_ = 0;
        calibration_timer_.restart();
        calibration_ = Calibration::SAMPLING;
    }

    if (calibration_ == Calibration::SAMPLING && calibration_timer_.elapsed() > 10.0f) {
        calibration_ = Calibration::FAILED;
        return;
    }

    if (calibration_ != Calibration::SAMPLING)
        return;

    // 用陀螺仪零偏和加速度模长判断静止窗口
    const Vector4 sample{imu_.gyro.x(), imu_.gyro.y(), imu_.gyro.z(), imu_.accel.norm()};
    if (calibration_samples_ == 0) {
        calibration_min_ = calibration_max_ = calibration_mean_ = sample;
    }
    calibration_samples_++;
    bool disturbed = std::abs(sample[3] - 9.81f) > 0.5f;
    for (int i = 0; i < 4; i++) {
        calibration_min_[i] = std::min(calibration_min_[i], sample[i]);
        calibration_max_[i] = std::max(calibration_max_[i], sample[i]);
        calibration_mean_[i] += (sample[i] - calibration_mean_[i]) / calibration_samples_;
        const float range = i < 3 ? 0.15f : 0.5f;
        disturbed |= calibration_max_[i] - calibration_min_[i] > range;
    }
    if (disturbed) {
        calibration_samples_ = 0;
    } else if (calibration_samples_ == 6000) {
        // 仅在静止窗口完整且平均零偏足够小时提交
        bool valid = true;
        for (int i = 0; i < 3; i++)
            valid &= std::abs(calibration_mean_[i]) <= 0.01f;
        if (valid) {
            config_.gyro_offset = calibration_mean_.head<3>();
            config_.gravity = calibration_mean_[3];
            calibration_ = Calibration::SUCCEEDED;
            ekf_.reset_bias();
        } else {
            calibration_samples_ = 0;
        }
    }
}
