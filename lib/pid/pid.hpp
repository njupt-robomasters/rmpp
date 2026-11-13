#pragma once

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

    Pid() = default;
    explicit Pid(const Config* config);

    // dt 为距上次计算的间隔，s，由持有方统一计时并限幅
    // 误差先低通再进 P、I、D；微分项再单独低通；输出超过 max_out 时限幅并按 track() 回算积分
    // derr 由调用方给出时不再差分，仍经微分低通；构造或 clear() 后首帧不差分
    float calculate(float dt, float err, std::optional<float> derr = std::nullopt);

    // 执行器实际采用 actual 后调用（功率衰减、电机限幅等）：积分扣除输出差额，只向零收缩不过零；输出同步为 actual
    void track(float actual);

    // 清空状态；持有方在不使用的分支调用，重新使用时从零开始
    void clear();

  private:
    const Config* config_ = nullptr;

  public:
    float out = 0;

  private:
    float p_out_ = 0, i_out_ = 0, d_out_ = 0;
    float err_ = 0, last_err_ = 0;
    bool has_last_err_ = false;
};
