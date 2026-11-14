/*
 * 基于 Wang Hongxi 的 QuaternionEKF V1.2.0（2022/3/8）重构
 * 保留六状态模型、残差门控和零偏估计；矩阵维度固定，无堆分配
 */
#include "quaternion_ekf.hpp"

#include <algorithm>
#include <cmath>

using Matrix3x6 = Eigen::Matrix<float, 3, 6>;
using Matrix6x3 = Eigen::Matrix<float, 6, 3>;

QuaternionEkf::QuaternionEkf(Imu& imu, const Config& config)
    : Attitude(imu),
      quaternion_noise(config.quaternion_noise),
      bias_noise(config.bias_noise),
      accel_noise(config.accel_noise),
      bias_fading(config.bias_fading),
      innovation_threshold(config.innovation_threshold) {
    reset_bias();
}

void QuaternionEkf::reset_bias() {
    bias.setZero();
    converged = false;
    rejected_count = 0;
    covariance.setConstant(0.1f);
    for (int i = 0; i < 6; ++i)
        covariance(i, i) = i < 4 ? 100000.0f : 100.0f;
}

void QuaternionEkf::solve(const Vector3f& gyro, const Vector3f& accel) {
    const float dt = std::clamp(solve_timer.restart(), DT_MIN, DT_MAX);

    // 扣除剩余零偏后，恒角速度在 dt 内的旋转增量；极小角用一阶近似，避免除零
    const Vector3f rate{gyro.x() - bias.x(), gyro.y() - bias.y(), gyro.z()};
    const float rate_norm = rate.norm();
    const float half_angle = 0.5f * dt * rate_norm;
    if (!std::isfinite(half_angle))
        return;
    float scale = 0.5f * dt;
    if (half_angle > 1.0e-6f)
        scale = std::sin(half_angle) / rate_norm;
    const Quaternionf delta(std::cos(half_angle), scale * rate.x(), scale * rate.y(), scale * rate.z());

    const float accel_norm = accel.norm();
    const bool valid_accel = accel.allFinite() && std::isfinite(accel_norm) && accel_norm > 1.0e-6f;

    // 首个有效重力方向确定倾角，航向置零；该样本不再重复积分
    if (!initialized) {
        if (!valid_accel)
            return;
        const float roll = std::atan2(accel.y(), accel.z());
        const float pitch = std::atan2(-accel.x(), std::hypot(accel.y(), accel.z()));
        q = euler_to_quaternion({.pitch = pitch, .roll = roll});
        initialized = true;
        return;
    }

    // 预测：均值采用恒角速度的精确增量，零偏雅可比按周期一阶线性化
    const float w = q.w(), x = q.x(), y = q.y(), z = q.z();
    Quaternionf predicted_q = q * delta;
    if (!normalize_quaternion(predicted_q))
        return;

    // 左上 4×4 为右乘 δq 的矩阵，右上 4×2 为零偏对旧 q 的一阶影响，零偏自身保持
    const float dw = delta.w(), dx = delta.x(), dy = delta.y(), dz = delta.z();
    const float h = 0.5f * dt;
    Matrix6f transition;
    // clang-format off
    transition << dw, -dx, -dy, -dz,  x * h,  y * h,
                  dx,  dw,  dz, -dy, -w * h,  z * h,
                  dy, -dz,  dw,  dx, -z * h, -w * h,
                  dz,  dy, -dx,  dw,  y * h, -x * h,
                   0,   0,   0,   0,      1,      0,
                   0,   0,   0,   0,      0,      1;
    // clang-format on

    Matrix6f prior = covariance;
    for (int i = 4; i < 6; ++i)
        prior(i, i) = std::min(prior(i, i) / bias_fading, 10000.0f);
    Matrix6f predicted_covariance;
    predicted_covariance.noalias() = transition * prior * transition.transpose();
    for (int i = 0; i < 6; ++i)
        predicted_covariance(i, i) += (i < 4 ? quaternion_noise : bias_noise) * dt;
    if (!symmetrize_covariance(predicted_covariance))
        return;
    q = predicted_q;
    covariance = predicted_covariance;

    // 以下任一失败均保留有效预测
    if (!valid_accel)
        return;

    // h(q) = R(q)ᵀ [0, 0, 1]，观测为单位重力方向
    const float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    const Vector3f gravity{2 * (qx * qz - qw * qy),
                           2 * (qw * qx + qy * qz),
                           qw * qw - qx * qx - qy * qy + qz * qz};
    // H = ∂h/∂[q, bias]，零偏不直接影响观测
    Matrix3x6 observation;
    // clang-format off
    observation << -qy,  qz, -qw,  qx, 0, 0,
                    qx,  qw,  qz,  qy, 0, 0,
                    qw, -qx, -qy,  qz, 0, 0;
    // clang-format on
    observation.leftCols<4>() *= 2;
    const Vector3f residual = accel / accel_norm - gravity;
    const Matrix6x3 cross_covariance = covariance * observation.transpose();
    Matrix3f innovation = observation * cross_covariance;
    innovation.diagonal().array() += accel_noise;

    // 对称正定的 3×3 观测协方差，Cholesky 同时检查数值可解性
    // 由 S = L Lᵀ 求 S⁻¹r 与增益，避免显式求逆
    innovation = 0.5f * (innovation + innovation.transpose());
    const Eigen::LLT<Matrix3f> llt(innovation);
    if (llt.info() != Eigen::Success)
        return;
    const Vector3f weighted_residual = llt.solve(residual);
    if (!weighted_residual.allFinite())
        return;
    const float distance = residual.dot(weighted_residual);
    if (!std::isfinite(distance) || distance < 0)
        return;
    const float threshold = innovation_threshold;
    if (distance < 0.5f * threshold)
        converged = true;
    if (converged && distance > threshold) {
        const bool stable = rate_norm < 0.3f && std::abs(accel_norm - 9.81f) < 0.5f;
        if (stable)
            ++rejected_count;
        else
            rejected_count = 0;
        if (rejected_count <= 50)
            return;
        // 静稳时持续失配：保留姿态，重建不确定度后重新融合
        // 避免旧交叉协方差把大倾角误差误认为陀螺仪零偏
        reset_bias();
        return;
    }
    rejected_count = 0;
    float gain_scale = 1;
    if (converged && distance > 0.1f * threshold)
        gain_scale = (threshold - distance) / (0.9f * threshold);

    Matrix6x3 gain = llt.solve(cross_covariance.transpose()).transpose();
    if (!gain.allFinite())
        return;
    for (int i = 0; i < 6; ++i) {
        float scale = gain_scale;
        // 当轴接近重力方向时，降低该轴零偏的观测权重
        if (i >= 4)
            scale *= std::acos(std::clamp(std::abs(gravity[i - 4]), 0.0f, 1.0f)) / (0.5f * pi);
        gain.row(i) *= scale;
    }
    Eigen::Matrix<float, 6, 1> correction = gain * residual;
    for (int i = 0; i < 6; ++i) {
        // 同步缩放增益，使 Joseph 更新对应实际使用的零偏修正
        if (i >= 4 && converged && std::abs(correction[i]) > 0.01f * dt) {
            const float scale = 0.01f * dt / std::abs(correction[i]);
            correction[i] *= scale;
            gain.row(i) *= scale;
        }
        if (!std::isfinite(correction[i]))
            return;
    }
    Quaternionf corrected_q(q.w() + correction[0], q.x() + correction[1], q.y() + correction[2],
                            q.z() + correction[3]);
    if (!normalize_quaternion(corrected_q))
        return;
    if (!std::isfinite(bias.x() + correction[4]) || !std::isfinite(bias.y() + correction[5]))
        return;

    // Joseph 形式：P = (I-KH) P (I-KH)ᵀ + K R Kᵀ
    const Matrix6f remaining = Matrix6f::Identity() - gain * observation;
    Matrix6f posterior = remaining * covariance * remaining.transpose();
    posterior.noalias() += gain * accel_noise * gain.transpose();
    if (!symmetrize_covariance(posterior))
        return;
    q = corrected_q;
    covariance = posterior;
    bias += correction.tail<2>();
}

bool QuaternionEkf::normalize_quaternion(Quaternionf& q) {
    const float norm = q.norm();
    if (!std::isfinite(norm) || norm < 1.0e-6f)
        return false;
    q.normalize();
    return q.coeffs().allFinite();
}

bool QuaternionEkf::symmetrize_covariance(Matrix6f& covariance) {
    if (!covariance.allFinite())
        return false;
    for (int i = 0; i < 6; ++i)
        if (covariance(i, i) < 0)
            return false;
    covariance = 0.5f * (covariance + covariance.transpose());
    return covariance.allFinite();
}
