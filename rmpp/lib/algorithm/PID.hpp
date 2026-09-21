#pragma once

#include <optional>
#include "bsp/bsp.hpp"

class PID {
public:
    struct config_t {
        // Gains act on SI errors and seconds; fc is Hz, limits use the output unit.
        float kp{}, ki{}, kd{}; // p、i、d三参数
        float ff{};             // 前馈
        float max_i{};          // 积分限幅
        float max_out{};        // 输出限幅
        float fc{};             // 输出低通滤波截止频率
    };

    float out{}; // PID输出

    PID() = default;

    PID(const config_t* config);

    float Calculate(float err, std::optional<float> derr = std::nullopt);

    void Clear();

private:
    const config_t* config = nullptr; // PID参数

    float out_no_filter{}, p_out{}, i_out{}, d_out{}; // PID输出分量
    float err{}, last_err{};                          // 误差值 err = ref - measure
    BSP::Dwt dwt;                                   // 用于计算dt

    // 低通滤波
    static float lowpassFilter(float pre, float next, float fc, float dt);
};
