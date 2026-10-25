#include "pid.hpp"
#include <algorithm>
#include <cmath>
#include "math/unit.hpp"

Pid::Pid(const Config* config)
    : config_(config) {}

float Pid::calculate(float err, std::optional<float> derr) {
    if (config_ == nullptr)
        return 0.0f;
    const auto& config = *config_;
    const float dt = timer_.restart();

    this->err_ = err;
    p_out_ = config.kp * err;
    float di = config.ki * err * dt;
    if (derr.has_value())
        d_out_ = config.kd * derr.value();
    else
        d_out_ = config.kd * (err - last_err_) / dt;
    last_err_ = err;

    if (std::fabsf(out) >= config.max_out && di * i_out_ > 0)
        di = 0.0f;
    i_out_ += di;
    i_out_ = std::clamp(i_out_, -config.max_i, config.max_i);

    out_no_filter_ = p_out_ + i_out_ + d_out_ + config.ff;
    out = lowpass_filter(out, out_no_filter_, config.fc, dt);
    out = std::clamp(out, -config.max_out, config.max_out);
    return out;
}

void Pid::clear() {
    err_ = last_err_ = 0.0f;
    out = out_no_filter_ = p_out_ = i_out_ = d_out_ = 0.0f;
    timer_.restart();
}

float Pid::lowpass_filter(float pre, float next, float fc, float dt) {
    if (fc == 0)
        return next;
    const float rc = 1 / (2 * pi * fc);
    const float alpha = dt / (rc + dt);
    return pre * (1.0f - alpha) + next * alpha;
}
