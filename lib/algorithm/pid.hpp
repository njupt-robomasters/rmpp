#pragma once

#include "board/board.hpp"
#include <optional>

class Pid {
  public:
    struct Config {
        // Gains act on SI errors and seconds; fc is Hz, limits use the output unit.
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
    rmpp::Timer timer_;                                   // 用于计算dt

    // 低通滤波
    static float lowpass_filter(float pre, float next, float fc, float dt);
};
