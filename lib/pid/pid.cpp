#include "pid.hpp"
#include "math/filter.hpp"
#include <algorithm>
#include <cmath>

Pid::Pid(const Config* config)
    : config_(config) {}

float Pid::calculate(const float err, const std::optional<float> derr) {
    if (config_ == nullptr)
        return 0.0f;

    const auto& [kp, ki, kd, ff, max_i, max_out, fc] = *config_;
    const float dt = timer_.restart();

    err_ = err;
    p_out_ = kp * err;
    if (derr.has_value())
        d_out_ = kd * derr.value();
    else
        d_out_ = kd * (err - last_err_) / dt;
    last_err_ = err;

    i_out_ = std::clamp(i_out_ + ki * err * dt, -max_i, max_i);

    out_no_filter_ = p_out_ + i_out_ + d_out_ + ff;
    out = lowpass(out, out_no_filter_, fc, dt);

    // 输出限幅与外部限幅同样回算积分
    if (std::fabs(out) > max_out)
        track(std::clamp(out, -max_out, max_out));

    return out;
}

void Pid::track(const float actual) {
    if (config_ == nullptr)
        return;

    // 无低通时等价于将积分回算为 actual - P - D - ff；只向零收缩，避免大误差时反向累积拖慢响应
    const float i = i_out_ + actual - out;
    i_out_ = std::clamp(i, std::min(0.0f, i_out_), std::max(0.0f, i_out_));
    out = actual;
}

void Pid::clear() {
    err_ = last_err_ = 0.0f;
    out = out_no_filter_ = p_out_ = i_out_ = d_out_ = 0.0f;
    timer_.restart();
}
