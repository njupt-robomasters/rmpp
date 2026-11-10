#pragma once

#include "bsp/timer.hpp"
#include "math/eigen.hpp"
#include "pid/pid.hpp"

// 板载 BMI088：读取、零偏与安装方向修正、校准及恒温加热
class Imu {
  public:
    struct Config {
        Quaternionf orientation = Quaternionf::Identity(); // 传感器到机体
        Vector3f gyro_offset = Vector3f::Zero();           // 传感器系零偏，rad/s；校准成功后更新
        float gravity = 9.81f;                             // 标定时原始加速度模长，m/s²；校准成功后更新
        float temperature = 45.0f;                         // 目标温度，°C
        Pid::Config temperature_pid = {.kp = 0.25f, .ki = 0.01f, .max_i = 0.4f, .max_out = 1.0f};
    };

    // 校准及安装修正后的机体系量；读取成功时刷新
    Vector3f gyro = Vector3f::Zero();  // rad/s
    Vector3f accel = Vector3f::Zero(); // m/s²
    float temperature = 0;             // °C

    explicit Imu(const Config& config);

    // 板卡实现：首次调用初始化传感器，失败时下次重试；读取成功后执行一步加热闭环、推进校准并刷新输出
    // 读取失败时关闭加热，输出保持上次值
    void update();

    // 下次 update 开始 5 s 校准，非静止时提前失败；调用方负责热机和静止时机
    void start_calibration();

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
    float calibration_mean_[4] = {}; // 陀螺仪 xyz（rad/s）与加速度模长（m/s²）
    float calibration_min_[4] = {};
    float calibration_max_[4] = {};

    // 用传感器系原始量推进校准，成功时更新 gyro_offset 与 gravity
    void update_calibration(const Vector3f& gyro, const Vector3f& accel);
};
