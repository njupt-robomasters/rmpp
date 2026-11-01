#pragma once

#include "bsp/imu.hpp"
#include "bsp/timer.hpp"
#include "math/angle.hpp"
#include "math/eigen_types.hpp"
#include "quaternion_ekf.hpp"

// 姿态解算：安装方向修正、零偏校准、姿态算法和欧拉角输出
class Attitude {
  public:
    struct Config {
        Quaternion orientation = Quaternion::Identity(); // 传感器到机体
        Vector3 gyro_offset = Vector3::Zero();           // 传感器系零偏，rad/s；校准成功后更新
        float gravity = 9.81f;                           // 标定时原始加速度模长，m/s²；校准成功后更新
        QuaternionEkf::Config ekf;
    };

    // 校准及安装修正后的机体系量
    Vector3 gyro = Vector3::Zero();        // rad/s
    Vector3 accel = Vector3::Zero();       // m/s²
    Quaternion q = Quaternion::Identity(); // 机体到世界
    Angle yaw, pitch, roll;                // Z-Y-X 欧拉角，rad
    float yaw_total_angle = 0.0f;          // 累积偏航角，rad，不归一化

    Attitude(Imu& imu, const Config& config);

    // 读取 Imu 并更新姿态；读取失败时保持上次输出
    void update();

    // 立即开始采样；由 update 推进，调用方负责热机和静止时机
    void start_calibration();

    const Config& config() const {
        return config_;
    }

  private:
    Imu& imu_;
    Config config_;
    QuaternionEkf ekf_;

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

    void update_calibration();
};
