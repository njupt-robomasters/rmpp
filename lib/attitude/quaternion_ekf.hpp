#pragma once

#include "attitude.hpp"

// 固定六状态 EKF：机体到世界的四元数，以及机体 X/Y 陀螺仪剩余零偏
class QuaternionEkf : public Attitude {
  private:
    static constexpr float DT_MIN = 0.0001f, DT_MAX = 0.01f; // solve() 步长限幅，s；避免首次调用和停顿后的大步长

  public:
    struct Config {
        float quaternion_noise = 10.0f;       // 四元数过程噪声强度，>= 0
        float bias_noise = 0.001f;            // 零偏过程噪声强度，>= 0
        float accel_noise = 1.0e7f;           // 单位重力方向的观测方差，> 0
        float bias_fading = 1.0f;             // 每周期零偏协方差保留系数，(0, 1]
        float innovation_threshold = 1.0e-8f; // 归一化残差门限，> 0
    };

  private:
    Vector2f bias = Vector2f::Zero(); // rad/s；不估计 Z 零偏

    const Config config;

    // 跨周期收敛状态
    bool initialized = false;               // 已由首个有效重力方向确定初始倾角
    bool converged = false;                 // 残差曾低于半门限，此后启用门控与增益衰减
    unsigned rejected_count = 0;            // 静稳时连续超门限次数，超过 50 次重置零偏与协方差
    Matrix6f covariance = Matrix6f::Zero(); // 状态协方差，顺序 [qw qx qy qz bias_x bias_y]
    Timer solve_timer;                      // solve() 间隔

  public:
    QuaternionEkf(Imu& imu, const Config& config);

    // 清除剩余零偏并重建协方差，保留当前姿态
    void reset_bias();

  private:
    void solve(const Vector3f& gyro, const Vector3f& accel) override;

    // 归一化；模长过小或非有限时返回 false
    static bool normalize_quaternion(Quaternionf& q);

    // 检查有限且对角非负后对称化；不满足时返回 false
    static bool symmetrize_covariance(Matrix6f& covariance);
};
