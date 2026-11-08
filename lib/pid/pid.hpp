#pragma once

#include "bsp/timer.hpp"
#include <optional>

class Pid {
  public:
    struct Config {
        float kp = 0, ki = 0, kd = 0;
        float ff = 0;
        float max_i = 0;
        float max_out = 0;
        float err_fc = 0; // 误差输入低通截止频率，Hz，0 直通
        float d_fc = 0;   // 微分项低通截止频率，Hz，0 直通
    };

    float out = 0;

    Pid() = default;
    explicit Pid(const Config* config);

    // 误差先低通再进 P、I、D；微分项再单独低通；输出超过 max_out 时限幅并按 track() 回算积分
    // derr 由调用方给出时不再差分，仍经微分低通
    float calculate(float err, std::optional<float> derr = std::nullopt);

    // 执行器实际采用 actual 后调用（功率衰减、电机限幅等）：积分扣除输出差额，只向零收缩不过零；输出同步为 actual
    void track(float actual);

    void clear();

  private:
    const Config* config_ = nullptr;
    float err_ = 0, last_err_ = 0;
    float p_out_ = 0, i_out_ = 0, d_out_ = 0;
    Timer calculate_timer_{0.0001f, 0.01f};
};
