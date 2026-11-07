#pragma once

#include "bsp/timer.hpp"
#include <optional>

class Pid {
  public:
    struct Config {
        float kp{}, ki{}, kd{};
        float ff{};
        float max_i{};
        float max_out{};
        float fc{};
    };

    float out{};

    Pid() = default;
    explicit Pid(const Config* config);

    // 输出超过 max_out 时限幅并按 track() 回算积分
    float calculate(float err, std::optional<float> derr = std::nullopt);

    // 执行器实际采用 actual 后调用（功率衰减、电机限幅等）：积分扣除输出差额，只向零收缩不过零；输出同步为 actual
    void track(float actual);

    void clear();

  private:
    const Config* config_ = nullptr;
    float out_no_filter_{}, p_out_{}, i_out_{}, d_out_{};
    float err_{}, last_err_{};
    Timer timer_{0.0001f, 0.01f};
};
