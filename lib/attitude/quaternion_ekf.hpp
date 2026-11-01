#pragma once

#include "bsp/timer.hpp"
#include "math/eigen_types.hpp"

// 固定六状态：机体到世界的四元数，以及机体 X/Y 陀螺仪剩余零偏
class QuaternionEkf {
  public:
    struct Config {
        float quaternion_noise = 10.0f;       // 四元数过程噪声强度，>= 0
        float bias_noise = 0.001f;            // 零偏过程噪声强度，>= 0
        float accel_noise = 1.0e7f;           // 单位重力方向的观测方差，> 0
        float bias_fading = 1.0f;             // 每周期零偏协方差保留系数，(0, 1]
        float accel_filter_fc = 0.0f;         // 加速度低通截止频率，Hz，0 禁用
        float innovation_threshold = 1.0e-8f; // 归一化残差门限，> 0
    };

    Quaternion q = Quaternion::Identity(); // 机体到世界的单位四元数
    Vector2 bias = Vector2::Zero();        // rad/s；不估计 Z 零偏

    explicit QuaternionEkf(const Config& config);
    void update(const Vector3& gyro, const Vector3& accel);
    // 校准提交后清除剩余零偏并重建协方差，保留当前姿态
    void reset_bias();

    // 假定周期内角速度恒定；无效输入不改变 q，dt 单位 s
    static Quaternion integrate(const Quaternion& q, const Vector3& gyro, float dt);

  private:
    Config config_;
    Timer timer_{0.0001f, 0.01f};
    Matrix6 covariance_ = Matrix6::Zero();
    Vector3 filtered_accel_ = Vector3::Zero();
    bool initialized_ = false;
    bool converged_ = false;
    unsigned rejected_count_ = 0;
};
