#pragma once

#include "bsp/timer.hpp"
#include "math/eigen_types.hpp"
#include "pid/pid.hpp"

// 板载 BMI088：读取、零偏与安装方向修正、校准及恒温加热
class Imu {
  public:
    struct Config {
        Quaternion orientation = Quaternion::Identity(); // 传感器到机体
        Vector3 gyro_offset = Vector3::Zero();           // 传感器系零偏，rad/s；校准成功后更新
        float gravity = 9.81f;                           // 标定时原始加速度模长，m/s²；校准成功后更新
        float temperature = 45.0f;                       // 目标温度，°C
        Pid::Config temperature_pid = {.kp = 1.0f, .ki = 0.01f, .max_i = 0.4f, .max_out = 1.0f};
    };

    // 校准及安装修正后的机体系量；读取成功时刷新
    Vector3 gyro = Vector3::Zero();  // rad/s
    Vector3 accel = Vector3::Zero(); // m/s²
    float temperature = 0;           // °C

    Imu();
    explicit Imu(const Config& config);

    // 板卡实现：首次调用初始化传感器，失败时下次重试；读取成功后执行一步加热闭环、推进校准并刷新输出
    // 读取失败时关闭加热，输出保持上次值
    void update();

    // 下次 update 开始新的校准窗口；调用方负责热机和静止时机
    void start_calibration();

    const Config& config() const {
        return config_;
    }

  private:
    Config config_;
    Pid temperature_pid_;
    bool is_initialized_ = false;

    // 零偏校准
    enum class Calibration {
        IDLE,
        STARTING,
        SAMPLING,
        SUCCEEDED,
        FAILED
    } calibration_ = Calibration::IDLE;
    Timer calibration_timer_;
    uint32_t calibration_samples_ = 0;
    Vector4 calibration_mean_ = Vector4::Zero();
    Vector4 calibration_min_ = Vector4::Zero();
    Vector4 calibration_max_ = Vector4::Zero();

    // 用传感器系原始量推进校准，成功时更新 gyro_offset 与 gravity
    void update_calibration(const Vector3& gyro, const Vector3& accel);
};
