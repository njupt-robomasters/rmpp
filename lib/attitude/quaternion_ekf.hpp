#pragma once

#include "attitude.hpp"

// 固定六状态 EKF：机体到世界的四元数，以及机体 X/Y 陀螺仪剩余零偏
class QuaternionEkf : public Attitude {
  public:
    struct Config {
        float quaternion_noise = 10.0f;       // 四元数过程噪声强度，>= 0
        float bias_noise = 1.0e-3f;           // 零偏过程噪声强度，>= 0
        float accel_noise = 1.0e7f;           // 单位重力方向的观测方差，> 0
        float bias_fading = 1.0f;             // 每周期零偏协方差保留系数，(0, 1]
        float accel_filter_fc = 0.0f;         // 加速度低通截止频率，Hz，0 禁用
        float innovation_threshold = 1.0e-8f; // 归一化残差门限，> 0
    };

    Vector2f bias = Vector2f::Zero(); // rad/s；不估计 Z 零偏

    QuaternionEkf(Imu& imu, const Config& config);

    // 清除剩余零偏并重建协方差，保留当前姿态
    void reset_bias();

    // 假定周期内角速度恒定；无效输入不改变 q，dt 单位 s
    static Quaternionf integrate(const Quaternionf& q, const Vector3f& gyro, float dt);

  protected:
    void solve(const Vector3f& gyro, const Vector3f& accel) override;

  private:
    Config config_;
    Timer timer_{0.0001f, 0.01f};
    Matrix6f covariance_ = Matrix6f::Zero();
    Vector3f filtered_accel_ = Vector3f::Zero();
    bool initialized_ = false;
    bool converged_ = false;
    unsigned rejected_count_ = 0;
};
