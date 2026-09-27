#pragma once

#include "bsp/timer.hpp"
#include <optional>

class Pid {
  public:
    struct Config {
        // 增益按国际单位制误差和秒计算；fc 单位为 Hz，限幅使用输出量单位
        float kp{}, ki{}, kd{}; // p、i、d三参数
        float ff{};             // 前馈
        float max_i{};          // 积分限幅
        float max_out{};        // 输出限幅
        float fc{};             // 输出低通滤波截止频率
    };

    float out{}; // PID输出

    Pid() = default;

    Pid(const Config* config);

    float calculate(float err, std::optional<float> derr = std::nullopt);

    void clear();

  private:
    const Config* config_ = nullptr; // PID参数

    float out_no_filter_{}, p_out_{}, i_out_{}, d_out_{}; // PID输出分量
    float err_{}, last_err_{};                            // 误差值 err = ref - measure
    Timer timer_{0.0001f, 10.0f};                         // 保留控制周期限幅，避免首次或连续计算时微分除零

    // 低通滤波
    static float lowpass_filter(float pre, float next, float fc, float dt);
};
