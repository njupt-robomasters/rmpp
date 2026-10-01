/*
 * 基于 Wang Hongxi 的 QuaternionEKF V1.2.0（2022/3/8）重构
 * 保留六状态模型、残差门控和零偏估计；矩阵维度固定，无堆分配
 */
#include "quaternion_ekf.hpp"

#include "math/unit.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

template <int ROWS, int INNER, int COLS>
static void multiply(const float (&a)[ROWS][INNER], const float (&b)[INNER][COLS], float (&out)[ROWS][COLS]) {
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j) {
            float sum = 0;
            for (int k = 0; k < INNER; ++k)
                sum += a[i][k] * b[k][j];
            out[i][j] = sum;
        }
}

// out = a * bᵀ；输出不得与输入重叠
template <int ROWS, int INNER, int COLS>
static void multiply_transpose(const float (&a)[ROWS][INNER], const float (&b)[COLS][INNER], float (&out)[ROWS][COLS]) {
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j) {
            float sum = 0;
            for (int k = 0; k < INNER; ++k)
                sum += a[i][k] * b[j][k];
            out[i][j] = sum;
        }
}

// 已知 S = L Lᵀ，先前代再回代，避免显式求逆
static void solve_cholesky(const float lower[3][3], const float rhs[3], float out[3]) {
    for (int i = 0; i < 3; ++i) {
        float sum = rhs[i];
        for (int j = 0; j < i; ++j)
            sum -= lower[i][j] * out[j];
        out[i] = sum / lower[i][i];
    }
    for (int i = 2; i >= 0; --i) {
        float sum = out[i];
        for (int j = i + 1; j < 3; ++j)
            sum -= lower[j][i] * out[j];
        out[i] = sum / lower[i][i];
    }
}

static bool symmetrize_covariance(float covariance[6][6]) {
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(covariance[i][i]) || covariance[i][i] < 0)
            return false;
        for (int j = 0; j < i; ++j) {
            const float mean = 0.5f * covariance[i][j] + 0.5f * covariance[j][i];
            if (!std::isfinite(mean))
                return false;
            covariance[i][j] = covariance[j][i] = mean;
        }
    }
    return true;
}

static bool normalize_quaternion(float q[4]) {
    const float norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!std::isfinite(norm) || norm < 1.0e-6f)
        return false;
    for (int i = 0; i < 4; ++i)
        q[i] /= norm;
    return true;
}

static void multiply_quaternions(const float q[4], const float delta[4], float out[4]) {
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float dw = delta[0], dx = delta[1], dy = delta[2], dz = delta[3];
    out[0] = w * dw - x * dx - y * dy - z * dz;
    out[1] = x * dw + w * dx - z * dy + y * dz;
    out[2] = y * dw + z * dx + w * dy - x * dz;
    out[3] = z * dw - y * dx + x * dy + w * dz;
}

static bool rotation_increment(const float gyro[3], const float dt, float delta[4]) {
    if (!std::isfinite(dt) || dt <= 0)
        return false;
    const float norm = std::hypot(gyro[0], gyro[1], gyro[2]);
    const float half_angle = 0.5f * dt * norm;
    if (!std::isfinite(half_angle))
        return false;
    float scale = 0.5f * dt;
    if (half_angle > 1.0e-6f)
        scale = std::sin(half_angle) / norm;
    delta[0] = std::cos(half_angle);
    for (int i = 0; i < 3; ++i)
        delta[i + 1] = gyro[i] * scale;
    return true;
}

QuaternionEkf::QuaternionEkf(const Config& config) : config_(config) {
    reset_bias();
}

void QuaternionEkf::update(const float gyro[3], const float accel[3], const float dt) {
    const float rate[3]{gyro[0] - bias[0], gyro[1] - bias[1], gyro[2]};
    float delta[4];
    if (!rotation_increment(rate, dt, delta))
        return;
    const float accel_norm = std::hypot(accel[0], accel[1], accel[2]);
    const bool valid_accel = std::isfinite(accel_norm) && accel_norm > 1.0e-6f;

    // 首个有效重力方向确定倾角，航向置零；该样本不再重复积分
    if (!initialized_) {
        if (!valid_accel)
            return;
        const float roll = std::atan2(accel[1], accel[2]);
        const float pitch = std::atan2(-accel[0], std::hypot(accel[1], accel[2]));
        const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
        const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
        q[0] = cr * cp;
        q[1] = sr * cp;
        q[2] = cr * sp;
        q[3] = -sr * sp;
        std::memcpy(filtered_accel_, accel, sizeof(filtered_accel_));
        initialized_ = true;
        return;
    }

    // 预测：均值采用恒角速度的精确增量，零偏雅可比按周期一阶线性化
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float dw = delta[0], dx = delta[1], dy = delta[2], dz = delta[3];
    float predicted_q[4];
    multiply_quaternions(q, delta, predicted_q);
    if (!normalize_quaternion(predicted_q))
        return;
    const float half_dt = 0.5f * dt;
    const float transition[6][6]{
        {dw, -dx, -dy, -dz, x * half_dt, y * half_dt},
        {dx, dw, dz, -dy, -w * half_dt, z * half_dt},
        {dy, -dz, dw, dx, -z * half_dt, -w * half_dt},
        {dz, dy, -dx, dw, y * half_dt, -x * half_dt},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1},
    };
    float prior[6][6];
    std::memcpy(prior, covariance_, sizeof(prior));
    for (int i = 4; i < 6; ++i)
        prior[i][i] = std::min(prior[i][i] / config_.bias_fading, 10000.0f);
    float intermediate[6][6], predicted_covariance[6][6];
    multiply(transition, prior, intermediate);
    multiply_transpose(intermediate, transition, predicted_covariance);
    for (int i = 0; i < 6; ++i)
        predicted_covariance[i][i] += (i < 4 ? config_.quaternion_noise : config_.bias_noise) * dt;
    if (!symmetrize_covariance(predicted_covariance))
        return;
    std::memcpy(q, predicted_q, sizeof(q));
    std::memcpy(covariance_, predicted_covariance, sizeof(covariance_));

    // 以下任一失败均保留有效预测；无重力观测时不使用残留低通数据
    if (!valid_accel)
        return;
    const float alpha = dt / (config_.accel_filter_tau + dt);
    for (int i = 0; i < 3; ++i)
        filtered_accel_[i] += alpha * (accel[i] - filtered_accel_[i]);
    const float norm = std::hypot(filtered_accel_[0], filtered_accel_[1], filtered_accel_[2]);
    if (!std::isfinite(norm) || norm < 1.0e-6f)
        return;

    // h(q) = R(q)ᵀ [0, 0, 1]，观测为单位重力方向
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const float gravity[3]{2 * (qx * qz - qw * qy),
                           2 * (qw * qx + qy * qz),
                           qw * qw - qx * qx - qy * qy + qz * qz};
    const float observation[3][6]{
        {-2 * qy, 2 * qz, -2 * qw, 2 * qx, 0, 0},
        {2 * qx, 2 * qw, 2 * qz, 2 * qy, 0, 0},
        {2 * qw, -2 * qx, -2 * qy, 2 * qz, 0, 0},
    };
    float residual[3];
    for (int i = 0; i < 3; ++i)
        residual[i] = filtered_accel_[i] / norm - gravity[i];
    float cross_covariance[6][3], innovation[3][3];
    multiply_transpose(covariance_, observation, cross_covariance);
    multiply(observation, cross_covariance, innovation);
    for (int i = 0; i < 3; ++i)
        innovation[i][i] += config_.accel_noise;

    // 对称正定的 3×3 观测协方差，Cholesky 同时检查数值可解性
    float lower[3][3]{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j <= i; ++j) {
            float sum = 0.5f * innovation[i][j] + 0.5f * innovation[j][i];
            for (int k = 0; k < j; ++k)
                sum -= lower[i][k] * lower[j][k];
            if (!std::isfinite(sum))
                return;
            if (i == j) {
                if (sum <= 1.0e-7f * innovation[i][i])
                    return;
                lower[i][j] = std::sqrt(sum);
            } else {
                lower[i][j] = sum / lower[j][j];
            }
        }

    float weighted_residual[3];
    solve_cholesky(lower, residual, weighted_residual);
    float distance = 0;
    for (int i = 0; i < 3; ++i)
        distance += residual[i] * weighted_residual[i];
    if (!std::isfinite(distance) || distance < 0)
        return;
    const float threshold = config_.innovation_threshold;
    if (distance < 0.5f * threshold)
        converged_ = true;
    if (converged_ && distance > threshold) {
        const bool stable = std::hypot(rate[0], rate[1], rate[2]) < 0.3f && std::abs(norm - 9.81f) < 0.5f;
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

    float gain[6][3];
    for (int i = 0; i < 6; ++i) {
        solve_cholesky(lower, cross_covariance[i], gain[i]);
        float scale = gain_scale;
        // 当轴接近重力方向时，降低该轴零偏的观测权重
        if (i >= 4)
            scale *= std::acos(std::clamp(std::abs(gravity[i - 4]), 0.0f, 1.0f)) / (0.5f * pi);
        for (int j = 0; j < 3; ++j)
            gain[i][j] *= scale;
    }
    float correction[6]{};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j)
            correction[i] += gain[i][j] * residual[j];
        // 同步缩放增益，使 Joseph 更新对应实际使用的零偏修正
        if (i >= 4 && converged_ && std::abs(correction[i]) > 0.01f * dt) {
            const float scale = 0.01f * dt / std::abs(correction[i]);
            correction[i] *= scale;
            for (int j = 0; j < 3; ++j)
                gain[i][j] *= scale;
        }
        if (!std::isfinite(correction[i]))
            return;
    }
    float corrected_q[4];
    for (int i = 0; i < 4; ++i)
        corrected_q[i] = q[i] + correction[i];
    if (!normalize_quaternion(corrected_q))
        return;
    if (!std::isfinite(bias[0] + correction[4]) || !std::isfinite(bias[1] + correction[5]))
        return;

    // Joseph 形式：P = (I-KH) P (I-KH)ᵀ + K R Kᵀ
    float remaining[6][6], posterior[6][6];
    multiply(gain, observation, remaining);
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            remaining[i][j] = (i == j ? 1.0f : 0.0f) - remaining[i][j];
    multiply(remaining, covariance_, intermediate);
    multiply_transpose(intermediate, remaining, posterior);
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            for (int k = 0; k < 3; ++k)
                posterior[i][j] += gain[i][k] * config_.accel_noise * gain[j][k];
    if (!symmetrize_covariance(posterior))
        return;
    std::memcpy(q, corrected_q, sizeof(q));
    std::memcpy(covariance_, posterior, sizeof(covariance_));
    bias[0] += correction[4];
    bias[1] += correction[5];
}

void QuaternionEkf::reset_bias() {
    bias[0] = bias[1] = 0;
    converged_ = false;
    rejected_count_ = 0;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            covariance_[i][j] = 0.1f;
            if (i == j)
                covariance_[i][j] = i < 4 ? 100000.0f : 100.0f;
        }
}

void QuaternionEkf::integrate(float q[4], const float gyro[3], const float dt) {
    float delta[4];
    if (!rotation_increment(gyro, dt, delta))
        return;
    float next[4];
    multiply_quaternions(q, delta, next);
    if (normalize_quaternion(next))
        std::memcpy(q, next, sizeof(next));
}
