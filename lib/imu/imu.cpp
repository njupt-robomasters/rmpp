#include "imu.hpp"

#include <algorithm>
#include <cmath>

Imu::Imu(const Config& config)
    : config(config), sensor_(config.hardware.sensor),
      ekf_(config.control.ekf), temperature_pid_(&config.control.temperature_pid_config) {
}

void Imu::update() {
    // 初始化陀螺仪
    if (!initialized_) {
        if (!sensor_.init())
            return;
        initialized_ = true;
    }

    // 读取陀螺仪
    sensor_.update();

    update_heater();
    update_calibration();

    // 修正陀螺仪读数
    gyro = sensor_.gyro - config.adjust.gyro_offset;
    accel = sensor_.accel * (9.81f / config.adjust.gravity);
    gyro = config.adjust.orientation * gyro;
    accel = config.adjust.orientation * accel;

    // 陀螺仪ekf解算
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

void Imu::start_calibration() {
    calibration_ = Calibration::STARTING;
}

Quaternion Imu::euler_to_quaternion(const EulerAngles& euler) {
    const Eigen::AngleAxisf yaw(euler.yaw, Eigen::Vector3f::UnitZ());
    const Eigen::AngleAxisf pitch(euler.pitch, Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf roll(euler.roll, Eigen::Vector3f::UnitX());
    return yaw * pitch * roll;
}

EulerAngles Imu::quaternion_to_euler(const Quaternion& quaternion) {
    Quaternion q = quaternion;
    const float norm = q.norm();
    if (std::isfinite(norm) && norm > 1.0e-6f)
        q.normalize();
    const float yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                                 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    const float pitch = std::asin(std::clamp(2 * (q.w() * q.y() - q.x() * q.z()), -1.0f, 1.0f));
    const float roll = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()),
                                  1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    return {.yaw = Angle(yaw), .pitch = Angle(pitch), .roll = Angle(roll)};
}

Vector3 Imu::body_to_world(const Vector3& body, const Quaternion& q) {
    return q * body;
}

Vector3 Imu::world_to_body(const Vector3& world, const Quaternion& q) {
    return q.conjugate() * world;
}

void Imu::update_heater() {
    temperature = sensor_.temperature;
    heater_duty = std::clamp(temperature_pid_.calculate(config.control.temperature - temperature), 0.0f, 1.0f);
    config.hardware.heater.set_duty(heater_duty);
}

void Imu::update_calibration() {
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
    const Vector4 sample{sensor_.gyro.x(), sensor_.gyro.y(), sensor_.gyro.z(), sensor_.accel.norm()};
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
            config.adjust.gyro_offset = calibration_mean_.head<3>();
            config.adjust.gravity = calibration_mean_[3];
            calibration_ = Calibration::SUCCEEDED;
            ekf_.reset_bias();
        } else {
            calibration_samples_ = 0;
        }
    }
}
