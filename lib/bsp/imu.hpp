#pragma once

#include "math/eigen_types.hpp"
#include "pid/pid.hpp"

// 板载 BMI088 及其恒温加热
class Imu {
  public:
    struct Config {
        float temperature = 45.0f; // 目标温度，°C
        Pid::Config temperature_pid = {.kp = 1.0f, .ki = 0.01f, .max_i = 0.4f, .max_out = 1.0f};
    };

    // 传感器系原始量，尚未校准；update 返回 true 时刷新
    Vector3 gyro = Vector3::Zero();  // rad/s
    Vector3 accel = Vector3::Zero(); // m/s²
    float temperature = 0;           // °C

    Imu();
    explicit Imu(const Config& config);

    // 首次调用初始化传感器，失败时下次重试；读取成功后执行一步加热闭环，失败时关闭加热
    bool update();

    const Config& config() const {
        return config_;
    }

  private:
    Config config_;
    Pid temperature_pid_;
    bool is_initialized_ = false;
};
