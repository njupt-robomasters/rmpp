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

    float calculate(float err, std::optional<float> derr = std::nullopt);
    void clear();

  private:
    const Config* config_ = nullptr;
    float out_no_filter_{}, p_out_{}, i_out_{}, d_out_{};
    float err_{}, last_err_{};
    Timer timer_{0.0001f, 0.01f};

    static float lowpass_filter(float pre, float next, float fc, float dt);
};
