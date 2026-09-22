#include "pid.hpp"
#include "math/math.hpp"
#include "math/unit.hpp"
#include <cmath>

Pid::Pid(const Config* config) : config_(config) {}

float Pid::calculate(float err, std::optional<float> derr) {
    // 未设置PID参数，输出0
    if (config_ == nullptr)
        return 0.0f;
    const auto& config = *config_;

    // 计算dt
    const float dt = timer_.update_dt();

    // 保存误差
    this->err_ = err;

    // 基础PID计算
    p_out_ = config.kp * err;
    float di = config.ki * err * dt;
    if (derr.has_value()) {
        d_out_ = config.kd * derr.value(); // 使用给定微分
    } else {
        d_out_ = config.kd * (err - last_err_) / dt; // 计算微分
    }
    last_err_ = err; // 更新上一次误差，用于微分

    // 积分限幅
    // 情况一：总输出饱和，且I输出呈累计趋势 -> 阻止I继续累积
    if (std::fabsf(out) >= config.max_out) {
        if (di * i_out_ > 0) { // 积分呈累积趋势
            di = 0.0f;
        }
    }
    // 情况二：I输出超过设定最大值 -> 对I输出钳位
    i_out_ += di;
    i_out_ = clamp(i_out_, config.max_i);

    // 输出滤波
    out_no_filter_ = p_out_ + i_out_ + d_out_ + config.ff;
    out = lowpass_filter(out, out_no_filter_, config.fc, dt);

    // 输出限幅
    out = clamp(out, config.max_out);

    return out;
}

void Pid::clear() {
    err_ = last_err_ = 0.0f;
    out = out_no_filter_ = p_out_ = i_out_ = d_out_ = 0.0f;
    timer_.reset();
}

float Pid::lowpass_filter(float pre, float next, float fc, float dt) {
    if (fc == 0)
        return next; // 禁用滤波

    const float rc = 1 / (2 * pi * fc);
    const float alpha = dt / (rc + dt);
    const float out = pre * (1.0f - alpha) + next * alpha;
    return out;
}
