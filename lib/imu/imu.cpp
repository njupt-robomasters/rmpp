#include "imu.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

Imu::Imu(const Config& config)
    : config_(config), sensor_({.accel = config_.hardware.accel, .gyro = config_.hardware.gyro}),
      ekf_(config_.control.ekf), temperature_pid_(&config_.control.temperature_pid) {
    euler_to_quaternion(config_.adjust.yaw, config_.adjust.pitch, config_.adjust.roll, installation_q_);
}

const Imu::Config& Imu::config() const {
    return config_;
}

void Imu::update() {
    // 超时保留旧标定；通信失败也不延长总等待时间
    if (calibration == Calibration::SAMPLING && calibration_timer_.elapsed() > 10.0f)
        calibration = Calibration::FAILED;
    if (!initialized_) {
        if (!sensor_.init())
            return;
        initialized_ = true;
        timer_.restart();
        temperature_pid_.clear();
    }
    const float dt = timer_.restart();
    if (!sensor_.read()) {
        heater_duty = 0;
        config_.hardware.heater.set_duty(heater_duty);
        temperature_pid_.clear();
        return;
    }

    // 温控与校准共用本轮采样，校准期间继续维持温度和姿态输出
    temperature = sensor_.temperature;
    heater_duty = std::clamp(temperature_pid_.calculate(config_.control.temperature - temperature), 0.0f, 1.0f);
    config_.hardware.heater.set_duty(heater_duty);

    if (calibration == Calibration::SAMPLING) {
        const float sample[4]{sensor_.gyro[0], sensor_.gyro[1], sensor_.gyro[2],
                              std::hypot(sensor_.accel[0], sensor_.accel[1], sensor_.accel[2])};
        if (calibration_samples == 0) {
            std::memcpy(calibration_min_, sample, sizeof(sample));
            std::memcpy(calibration_max_, sample, sizeof(sample));
            std::memcpy(calibration_mean_, sample, sizeof(sample));
        }
        ++calibration_samples;
        bool disturbed = std::abs(sample[3] - 9.81f) > 0.5f;
        for (int i = 0; i < 4; ++i) {
            calibration_min_[i] = std::min(calibration_min_[i], sample[i]);
            calibration_max_[i] = std::max(calibration_max_[i], sample[i]);
            calibration_mean_[i] += (sample[i] - calibration_mean_[i]) / calibration_samples;
            const float range = i < 3 ? 0.15f : 0.5f;
            disturbed |= calibration_max_[i] - calibration_min_[i] > range;
        }
        if (disturbed) {
            calibration_samples = 0;
        } else if (calibration_samples == 6000) {
            bool valid = true;
            for (int i = 0; i < 3; ++i)
                valid &= std::abs(calibration_mean_[i]) <= 0.01f;
            if (valid) {
                for (int i = 0; i < 3; ++i)
                    config_.adjust.gyro_offset[i] = calibration_mean_[i];
                config_.adjust.gravity = calibration_mean_[3];
                calibration = Calibration::SUCCEEDED;
                ekf_.reset_bias();
            } else {
                calibration_samples = 0;
            }
        }
    }

    // 先在传感器坐标系校准，再旋转到机体坐标系
    for (int i = 0; i < 3; ++i) {
        gyro[i] = sensor_.gyro[i] - config_.adjust.gyro_offset[i];
        accel[i] = sensor_.accel[i] * (9.81f / config_.adjust.gravity);
    }
    body_to_world(gyro, gyro, installation_q_);
    body_to_world(accel, accel, installation_q_);
    ekf_.update(gyro, accel, dt);
    std::memcpy(q, ekf_.q, sizeof(q));

    const Angle last_yaw = yaw;
    std::tie(yaw, pitch, roll) = quaternion_to_euler(q);
    yaw_total_angle += Angle(yaw - last_yaw);
}

void Imu::start_calibration() {
    calibration = Calibration::SAMPLING;
    calibration_samples = 0;
    calibration_timer_.restart();
}

void Imu::euler_to_quaternion(const Angle yaw, const Angle pitch, const Angle roll, float q[4]) {
    const float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
    const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

std::tuple<Angle, Angle, Angle> Imu::quaternion_to_euler(const float q[4]) {
    const Angle yaw = std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
    const Angle pitch = std::asin(std::clamp(2 * (q[0] * q[2] - q[1] * q[3]), -1.0f, 1.0f));
    const Angle roll = std::atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    return {yaw, pitch, roll};
}

void Imu::body_to_world(const float body[3], float world[3], const float q[4]) {
    const float x = body[0], y = body[1], z = body[2];
    world[0] = (1 - 2 * (q[2] * q[2] + q[3] * q[3])) * x +
               2 * (q[1] * q[2] - q[0] * q[3]) * y + 2 * (q[1] * q[3] + q[0] * q[2]) * z;
    world[1] = 2 * (q[1] * q[2] + q[0] * q[3]) * x +
               (1 - 2 * (q[1] * q[1] + q[3] * q[3])) * y + 2 * (q[2] * q[3] - q[0] * q[1]) * z;
    world[2] = 2 * (q[1] * q[3] - q[0] * q[2]) * x +
               2 * (q[2] * q[3] + q[0] * q[1]) * y + (1 - 2 * (q[1] * q[1] + q[2] * q[2])) * z;
}

void Imu::world_to_body(const float world[3], float body[3], const float q[4]) {
    const float conjugate[4]{q[0], -q[1], -q[2], -q[3]};
    body_to_world(world, body, conjugate);
}

void Imu::integrate_quaternion(float q[4], const float gx, const float gy, const float gz, const float dt) {
    const float gyro[3]{gx, gy, gz};
    QuaternionEkf::integrate(q, gyro, dt);
}
