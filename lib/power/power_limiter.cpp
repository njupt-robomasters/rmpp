#include "power_limiter.hpp"
#include "math/filter.hpp"
#include <algorithm>
#include <cmath>

PowerLimiter::PowerLimiter(SuperCap* const super_cap, const Config& config)
    : super_cap_(super_cap), config_(config) {}

void PowerLimiter::set_limit(const float limit) {
    limit_ = limit;
}

void PowerLimiter::set_buffer_energy(const float buffer_energy) {
    buffer_energy_ = buffer_energy;
}

void PowerLimiter::update(Motor* const wheel[4]) {
    const float dt = timer_.restart();

    // 功率估计，离线轮反馈已清零
    const float k = 0.75f * config_.wheel_resistance;
    estimate = 0;
    for (int i = 0; i < 4; i++)
        estimate += wheel[i]->torque * wheel[i]->speed + k * wheel[i]->current * wheel[i]->current;

    // 功率预算：缓冲能量 30 J 为零、60 J 为 buffer_gain_max 倍裁判上限，其间线性
    // 测量偏小时缓冲能量下降、预算随之降低，稳定在预算等于真实上限处
    const float ratio = std::clamp((buffer_energy_ - BUFFER_CUTOFF) / (BUFFER_FULL - BUFFER_CUTOFF), 0.0f, 1.0f);
    budget = limit_ * config_.buffer_gain_max * ratio;

    // 超电：电池侧目标取功率预算，不限功率时发协议最大值
    if (super_cap_ != nullptr) {
        super_cap_->update();
        super_cap_->set_battery_power(limit_ == 65535 ? limit_ : budget);
    }

    // 本周期上限：超电生效时用实测修正估计偏差，BOOST 叠加电容可补偿功率
    max = budget;
    if (super_cap_ != nullptr && super_cap_->active_mode != SuperCap::OFF) {
        measure = super_cap_->power.out;
        error = lowpass(error, measure - estimate, config_.error_fc, dt);
        max += super_cap_->power.boost;
    } else {
        measure = 0;
        error = 0;
    }
}

void PowerLimiter::control(Motor* const wheel[4], const Vector4f& current_xy, const Vector4f& current_z, const bool is_spin) {
    gain_xy = gain_z = 1;

    // 裁判系统不限功率时读出 65535，此时缓冲能量也异常，跳过
    if (limit_ != 65535) {
        // 预测功率按两组系数展开：二次项、交叉项与一次项系数
        const float k = 0.75f * config_.wheel_resistance;
        float xx = 0, zz = 0, xz = 0, x = 0, z = 0;
        for (int i = 0; i < 4; i++) {
            const float kt_speed = wheel[i]->config.drive.kt * wheel[i]->speed;
            xx += k * current_xy[i] * current_xy[i];
            zz += k * current_z[i] * current_z[i];
            xz += 2 * k * current_xy[i] * current_z[i];
            x += kt_speed * current_xy[i];
            z += kt_speed * current_z[i];
        }
        const float c = error - max;

        if (is_spin) {
            // 平移不衰减解旋转；停止旋转仍超功率时再解平移
            gain_z = solve_gain(zz, xz + z, xx + x + c);
            if (gain_z == 0)
                gain_xy = solve_gain(xx, x, c);
        } else {
            gain_xy = gain_z = solve_gain(xx + zz + xz, x + z, c);
        }
    }

    for (int i = 0; i < 4; i++)
        wheel[i]->set_current(gain_xy * current_xy[i] + gain_z * current_z[i]);
}

float PowerLimiter::solve_gain(const float a, const float b, const float c) {
    if (a + b + c <= 0)
        return 1;
    if (a > 0) {
        const float delta = b * b - 4 * a * c;
        if (delta < 0)
            return 0;
        return std::clamp((-b + std::sqrt(delta)) / (2 * a), 0.0f, 1.0f);
    }
    if (b > 0) // 零电流时为一次式
        return std::clamp(-c / b, 0.0f, 1.0f);
    return 0;
}
