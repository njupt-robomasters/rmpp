#include "PID.hpp"
#include <cmath>
#include "math/Conversions.hpp"
#include "math/Math.hpp"

#define kp (config->kp)
#define ki (config->ki)
#define kd (config->kd)
#define ff (config->ff)
#define max_i (config->max_i)
#define max_out (config->max_out)
#define fc (config->fc)

PID::PID(const config_t* config) : config(config) {}

float PID::Calculate(float err, std::optional<float> derr) {
    // 未设置PID参数，输出0
    if (config == nullptr)
        return 0.0f;

    // 计算dt
    const float dt = dwt.UpdateDT();

    // 保存误差
    this->err = err;

    // 基础PID计算
    p_out = kp * err;
    float di = ki * err * dt;
    if (derr.has_value()) {
        d_out = kd * derr.value(); // 使用给定微分
    } else {
        d_out = kd * (err - last_err) / dt; // 计算微分
    }
    last_err = err; // 更新上一次误差，用于微分

    // 积分限幅
    // 情况一：总输出饱和，且I输出呈累计趋势 -> 阻止I继续累积
    if (std::fabsf(out) >= max_out) {
        if (di * i_out > 0) { // 积分呈累积趋势
            di = 0.0f;
        }
    }
    // 情况二：I输出超过设定最大值 -> 对I输出钳位
    i_out += di;
    i_out = math::clamp(i_out, max_i);

    // 输出滤波
    out_no_filter = p_out + i_out + d_out + ff;
    out = lowpassFilter(out, out_no_filter, fc, dt);

    // 输出限幅
    out = math::clamp(out, max_out);

    return out;
}

void PID::Clear() {
    err = last_err = 0.0f;
    out = out_no_filter = p_out = i_out = d_out = 0.0f;
    dwt.Reset();
}

#undef fc

float PID::lowpassFilter(float pre, float next, float fc, float dt) {
    if (fc == 0) return next; // 禁用滤波

    const float rc = 1 / (2 * math::pi * fc);
    const float alpha = dt / (rc + dt);
    const float out = pre * (1.0f - alpha) + next * alpha;
    return out;
}