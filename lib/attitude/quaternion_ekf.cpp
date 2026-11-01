/*
 * 基于 Wang Hongxi 的 QuaternionEKF V1.2.0（2022/3/8）重构
 * 保留六状态模型、残差门控和零偏估计；矩阵维度固定，无堆分配
 */
#include "quaternion_ekf.hpp"

#include "math/filter.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>

using Matrix3x6 = Eigen::Matrix<float, 3, 6>;
using Matrix6x3 = Eigen::Matrix<float, 6, 3>;

static Matrix6 multiply(const Matrix6& a, const Matrix6& b) {
    Matrix6 out;
    out.noalias() = a * b;
    return out;
}

static Matrix6 multiply_transpose(const Matrix6& a, const Matrix6& b) {
    Matrix6 out;
    out.noalias() = a * b.transpose();
    return out;
}

// 已知 S = L Lᵀ，先前代再回代，避免显式求逆
static bool solve_cholesky(const Matrix3& lower, const Vector3& rhs, Vector3& out) {
    out = lower.template triangularView<Eigen::Lower>().solve(rhs);
    out = lower.transpose().template triangularView<Eigen::Upper>().solve(out);
    return out.allFinite();
}

static bool symmetrize_covariance(Matrix6& covariance) {
    if (!covariance.allFinite())
        return false;
    for (int i = 0; i < 6; ++i)
        if (covariance(i, i) < 0)
            return false;
    covariance = 0.5f * (covariance + covariance.transpose());
    return covariance.allFinite();
}

static bool normalize_quaternion(Quaternion& q) {
    const float norm = q.norm();
    if (!std::isfinite(norm) || norm < 1.0e-6f)
        return false;
    q.normalize();
    return q.coeffs().allFinite();
}

static bool rotation_increment(const Vector3& gyro, const float dt, Quaternion& delta) {
    if (!gyro.allFinite() || !std::isfinite(dt) || dt <= 0)
        return false;
    const float norm = gyro.norm();
    const float half_angle = 0.5f * dt * norm;
    if (!std::isfinite(half_angle))
        return false;
    float scale = 0.5f * dt;
    if (half_angle > 1.0e-6f)
        scale = std::sin(half_angle) / norm;
    delta = Quaternion(std::cos(half_angle), scale * gyro.x(), scale * gyro.y(), scale * gyro.z());
    return delta.coeffs().allFinite();
}

QuaternionEkf::QuaternionEkf(Imu& imu)
    : QuaternionEkf(imu, Config{}) {}

QuaternionEkf::QuaternionEkf(Imu& imu, const Config& config)
    : Attitude(imu), config_(config) {
    reset_bias();
}

void QuaternionEkf::solve(const Vector3& gyro, const Vector3& accel) {
    const float dt = timer_.restart();
    const Vector3 rate{gyro.x() - bias.x(), gyro.y() - bias.y(), gyro.z()};
    Quaternion delta;
    if (!rotation_increment(rate, dt, delta))
        return;
    const float accel_norm = accel.norm();
    const bool valid_accel = accel.allFinite() && std::isfinite(accel_norm) && accel_norm > 1.0e-6f;

    // 首个有效重力方向确定倾角，航向置零；该样本不再重复积分
    if (!initialized_) {
        if (!valid_accel)
            return;
        const float roll = std::atan2(accel.y(), accel.z());
        const float pitch = std::atan2(-accel.x(), std::hypot(accel.y(), accel.z()));
        const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
        const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
        q = Quaternion(cr * cp, sr * cp, cr * sp, -sr * sp);
        filtered_accel_ = accel;
        initialized_ = true;
        return;
    }

    // 预测：均值采用恒角速度的精确增量，零偏雅可比按周期一阶线性化
    const float w = q.w(), x = q.x(), y = q.y(), z = q.z();
    Quaternion predicted_q = q * delta;
    if (!normalize_quaternion(predicted_q))
        return;
    const float half_dt = 0.5f * dt;
    Matrix6 transition = Matrix6::Zero();
    transition(0, 0) = delta.w();
    transition(0, 1) = -delta.x();
    transition(0, 2) = -delta.y();
    transition(0, 3) = -delta.z();
    transition(0, 4) = x * half_dt;
    transition(0, 5) = y * half_dt;
    transition(1, 0) = delta.x();
    transition(1, 1) = delta.w();
    transition(1, 2) = delta.z();
    transition(1, 3) = -delta.y();
    transition(1, 4) = -w * half_dt;
    transition(1, 5) = z * half_dt;
    transition(2, 0) = delta.y();
    transition(2, 1) = -delta.z();
    transition(2, 2) = delta.w();
    transition(2, 3) = delta.x();
    transition(2, 4) = -z * half_dt;
    transition(2, 5) = -w * half_dt;
    transition(3, 0) = delta.z();
    transition(3, 1) = delta.y();
    transition(3, 2) = -delta.x();
    transition(3, 3) = delta.w();
    transition(3, 4) = y * half_dt;
    transition(3, 5) = -x * half_dt;
    transition(4, 4) = 1;
    transition(5, 5) = 1;

    Matrix6 prior = covariance_;
    for (int i = 4; i < 6; ++i)
        prior(i, i) = std::min(prior(i, i) / config_.bias_fading, 10000.0f);
    Matrix6 predicted_covariance = multiply(transition, prior);
    predicted_covariance = multiply_transpose(predicted_covariance, transition);
    for (int i = 0; i < 6; ++i)
        predicted_covariance(i, i) += (i < 4 ? config_.quaternion_noise : config_.bias_noise) * dt;
    if (!symmetrize_covariance(predicted_covariance))
        return;
    q = predicted_q;
    covariance_ = predicted_covariance;

    // 以下任一失败均保留有效预测；无重力观测时不使用残留低通数据
    if (!valid_accel)
        return;
    filtered_accel_ = lowpass(filtered_accel_, accel, config_.accel_filter_fc, dt);
    const float norm = filtered_accel_.norm();
    if (!std::isfinite(norm) || norm < 1.0e-6f)
        return;

    // h(q) = R(q)ᵀ [0, 0, 1]，观测为单位重力方向
    const float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    const Vector3 gravity{2 * (qx * qz - qw * qy),
                          2 * (qw * qx + qy * qz),
                          qw * qw - qx * qx - qy * qy + qz * qz};
    Matrix3x6 observation = Matrix3x6::Zero();
    observation(0, 0) = -2 * qy;
    observation(0, 1) = 2 * qz;
    observation(0, 2) = -2 * qw;
    observation(0, 3) = 2 * qx;
    observation(1, 0) = 2 * qx;
    observation(1, 1) = 2 * qw;
    observation(1, 2) = 2 * qz;
    observation(1, 3) = 2 * qy;
    observation(2, 0) = 2 * qw;
    observation(2, 1) = -2 * qx;
    observation(2, 2) = -2 * qy;
    observation(2, 3) = 2 * qz;
    const Vector3 residual = filtered_accel_ / norm - gravity;
    const Matrix6x3 cross_covariance = covariance_ * observation.transpose();
    Matrix3 innovation = observation * cross_covariance;
    innovation.diagonal().array() += config_.accel_noise;

    // 对称正定的 3×3 观测协方差，Cholesky 同时检查数值可解性
    innovation = 0.5f * (innovation + innovation.transpose());
    Eigen::LLT<Matrix3> llt(innovation);
    if (llt.info() != Eigen::Success)
        return;
    const Matrix3 lower = llt.matrixL();
    Vector3 weighted_residual;
    if (!solve_cholesky(lower, residual, weighted_residual))
        return;
    const float distance = residual.dot(weighted_residual);
    if (!std::isfinite(distance) || distance < 0)
        return;
    const float threshold = config_.innovation_threshold;
    if (distance < 0.5f * threshold)
        converged_ = true;
    if (converged_ && distance > threshold) {
        const bool stable = rate.norm() < 0.3f && std::abs(norm - 9.81f) < 0.5f;
        if (stable)
            ++rejected_count_;
        else
            rejected_count_ = 0;
        if (rejected_count_ <= 50)
            return;
        // 静稳时持续失配：保留姿态，重建不确定度后重新融合
        // 避免旧交叉协方差把大倾角误差误认为陀螺仪零偏
        reset_bias();
        return;
    }
    rejected_count_ = 0;
    float gain_scale = 1;
    if (converged_ && distance > 0.1f * threshold)
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
        if (i >= 4 && converged_ && std::abs(correction[i]) > 0.01f * dt) {
            const float scale = 0.01f * dt / std::abs(correction[i]);
            correction[i] *= scale;
            gain.row(i) *= scale;
        }
        if (!std::isfinite(correction[i]))
            return;
    }
    Quaternion corrected_q(q.w() + correction[0], q.x() + correction[1], q.y() + correction[2],
                           q.z() + correction[3]);
    if (!normalize_quaternion(corrected_q))
        return;
    if (!std::isfinite(bias.x() + correction[4]) || !std::isfinite(bias.y() + correction[5]))
        return;

    // Joseph 形式：P = (I-KH) P (I-KH)ᵀ + K R Kᵀ
    const Eigen::Matrix<float, 6, 6> remaining = Eigen::Matrix<float, 6, 6>::Identity() - gain * observation;
    Matrix6 posterior = remaining * covariance_ * remaining.transpose();
    posterior.noalias() += gain * config_.accel_noise * gain.transpose();
    if (!symmetrize_covariance(posterior))
        return;
    q = corrected_q;
    covariance_ = posterior;
    bias += correction.tail<2>();
}

void QuaternionEkf::reset_bias() {
    bias.setZero();
    converged_ = false;
    rejected_count_ = 0;
    covariance_.setConstant(0.1f);
    for (int i = 0; i < 6; ++i)
        covariance_(i, i) = i < 4 ? 100000.0f : 100.0f;
}

Quaternion QuaternionEkf::integrate(const Quaternion& q, const Vector3& gyro, const float dt) {
    Quaternion delta;
    if (!rotation_increment(gyro, dt, delta))
        return q;
    Quaternion next = q * delta;
    if (!normalize_quaternion(next))
        return q;
    return next;
}
