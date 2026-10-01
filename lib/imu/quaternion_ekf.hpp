#pragma once

// 固定六状态：机体到世界的四元数，以及机体 X/Y 陀螺仪剩余零偏
class QuaternionEkf {
  public:
    struct Config {
        float quaternion_noise = 10.0f;       // 四元数过程噪声强度，>= 0
        float bias_noise = 0.001f;            // 零偏过程噪声强度，>= 0
        float accel_noise = 1.0e7f;           // 单位重力方向的观测方差，> 0
        float bias_fading = 1.0f;             // 每周期零偏协方差保留系数，(0, 1]
        float accel_filter_tau = 0.0f;        // 加速度低通时间常数，s，0 禁用
        float innovation_threshold = 1.0e-8f; // 归一化残差门限，> 0
    };

    float q[4]{1, 0, 0, 0}; // [w, x, y, z]，单位四元数
    float bias[2]{};        // rad/s；没有绝对航向源，不估计 Z 零偏

    explicit QuaternionEkf(const Config& config);
    void update(const float gyro[3], const float accel[3], float dt);
    // 校准提交后清除剩余零偏并重建协方差，保留当前姿态
    void reset_bias();

    // 假定周期内角速度恒定；无效输入不改变 q，dt 单位 s
    static void integrate(float q[4], const float gyro[3], float dt);

  private:
    Config config_;
    float covariance_[6][6]{};
    float filtered_accel_[3]{};
    bool initialized_ = false;
    bool converged_ = false;
    unsigned rejected_count_ = 0;
};
