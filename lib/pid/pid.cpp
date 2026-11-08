#include "pid.hpp"
#include "math/filter.hpp"
#include <algorithm>
#include <cmath>

Pid::Pid(const Config* config)
    : config_(config) {}

float Pid::calculate(const float err, const std::optional<float> derr) {
    if (config_ == nullptr)
        return 0;

    const auto& [kp, ki, kd, ff, max_i, max_out, err_fc, d_fc] = *config_;
    const float dt = calculate_timer_.restart();

    // 输入低通
    err_ = lowpass(err_, err, err_fc, dt);

    // P、I、D，微分项单独低通
    p_out_ = kp * err_;
    i_out_ = std::clamp(i_out_ + ki * err_ * dt, -max_i, max_i);
    const float d = derr.has_value() ? kd * derr.value() : kd * (err_ - last_err_) / dt;
    d_out_ = lowpass(d_out_, d, d_fc, dt);
    last_err_ = err_;

    // 输出限幅与外部限幅同样回算积分
    out = p_out_ + i_out_ + d_out_ + ff;
    if (std::fabs(out) > max_out)
        track(std::clamp(out, -max_out, max_out));

    return out;
}

void Pid::track(const float actual) {
    if (config_ == nullptr)
        return;

    // 积分回算为 actual - P - D - ff；只向零收缩，避免大误差时反向累积拖慢响应
    const float i = i_out_ + actual - out;
    i_out_ = std::clamp(i, std::min(0.0f, i_out_), std::max(0.0f, i_out_));
    out = actual;
}

void Pid::clear() {
    err_ = last_err_ = 0;
    out = p_out_ = i_out_ = d_out_ = 0;
    calculate_timer_.restart();
}
