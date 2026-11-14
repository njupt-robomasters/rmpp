#include "pid.hpp"
#include "math/filter.hpp"
#include <algorithm>
#include <cmath>

Pid::Pid(const Config* config)
    : config(config) {}

float Pid::calculate(const float dt, const float err, const std::optional<float> derr) {
    if (config == nullptr)
        return 0;

    const auto& [kp, ki, kd, ff, max_i, max_out, err_fc, d_fc] = *config;

    // 输入低通
    this->err = lowpass(this->err, err, err_fc, dt);

    // P、I、D，微分项单独低通；首帧没有上次误差，不差分
    p_out = kp * this->err;
    i_out = std::clamp(i_out + ki * this->err * dt, -max_i, max_i);
    float d = 0;
    if (derr.has_value())
        d = kd * derr.value();
    else if (has_last_err)
        d = kd * (this->err - last_err) / dt;
    d_out = lowpass(d_out, d, d_fc, dt);
    last_err = this->err;
    has_last_err = true;

    // 输出限幅与外部限幅同样回算积分
    out = p_out + i_out + d_out + ff;
    if (std::fabs(out) > max_out)
        track(std::clamp(out, -max_out, max_out));

    return out;
}

void Pid::track(const float actual) {
    if (config == nullptr)
        return;

    // 积分回算为 actual - P - D - ff；只向零收缩，避免大误差时反向累积拖慢响应
    const float i = i_out + actual - out;
    i_out = std::clamp(i, std::min(0.0f, i_out), std::max(0.0f, i_out));
    out = actual;
}

void Pid::clear() {
    err = last_err = 0;
    has_last_err = false;
    out = p_out = i_out = d_out = 0;
}
