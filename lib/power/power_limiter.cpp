#include "power_limiter.hpp"
#include "math/filter.hpp"
#include <algorithm>
#include <cmath>

PowerLimiter::PowerLimiter(const Config& config)
    : config_(config) {}

void PowerLimiter::set_limit(const float limit) {
    limit_ = limit;
}

void PowerLimiter::set_buffer_energy(const float buffer_energy) {
    buffer_energy_ = buffer_energy;
}

void PowerLimiter::update(const std::array<Motor*, 4>& wheel) {
    const float dt = timer_.restart();

    // 功率估计，离线轮反馈已清零
    const float k = 0.75f * config_.wheel_resistance;
    estimate = 0;
    for (const Motor* w : wheel)
        estimate += w->torque * w->speed + k * w->current * w->current;

    // 超电：电池侧目标取裁判上限减裕量
    SuperCap* const super_cap = config_.super_cap;
    if (super_cap != nullptr) {
        super_cap->update();
        super_cap->set_battery_power(limit_ - super_cap->config().power_margin);
    }

    // 功率反馈与上限：超电生效时用实测修正估计偏差，BOOST 叠加最大补偿功率；否则按估计与裁判上限
    if (super_cap != nullptr && super_cap->active_mode != SuperCap::OFF) {
        const SuperCap::Config& cap_config = super_cap->config();
        measure = super_cap->power.out;
        error = lowpass(error, measure - estimate, config_.error_fc, dt);
        max = limit_ - cap_config.power_margin;
        if (super_cap->active_mode == SuperCap::BOOST)
            max += cap_config.max_discharge_current * super_cap->voltage.cap;
    } else {
        measure = 0;
        error = 0;
        max = limit_;
    }
}

void PowerLimiter::control(const std::array<Motor*, 4>& wheel, const Vector4f& current_xy, const Vector4f& current_z, const bool is_spin) {
    gain_xy = gain_z = 1;

    // 裁判系统不限功率时读出 65535，此时缓冲能量也异常，跳过
    if (limit_ != 65535) {
        // 缓冲能量折算上限：30 J 以下为零，60 J 以上不衰减
        const float power_max = max * std::clamp((buffer_energy_ - 30) / 30, 0.0f, 1.0f);

        // 预测功率按两组系数展开：二次项、交叉项与一次项系数
        const float k = 0.75f * config_.wheel_resistance;
        float xx = 0, zz = 0, xz = 0, x = 0, z = 0;
        for (size_t i = 0; i < wheel.size(); i++) {
            const float kt_speed = wheel[i]->config.drive.kt * wheel[i]->speed;
            xx += k * current_xy[i] * current_xy[i];
            zz += k * current_z[i] * current_z[i];
            xz += 2 * k * current_xy[i] * current_z[i];
            x += kt_speed * current_xy[i];
            z += kt_speed * current_z[i];
        }
        const float c = error - power_max;

        if (is_spin) {
            // 平移不衰减解旋转；停止旋转仍超功率时再解平移
            gain_z = solve_gain(zz, xz + z, xx + x + c);
            if (gain_z == 0)
                gain_xy = solve_gain(xx, x, c);
        } else {
            gain_xy = gain_z = solve_gain(xx + zz + xz, x + z, c);
        }
    }

    for (size_t i = 0; i < wheel.size(); i++)
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
