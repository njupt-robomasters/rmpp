#pragma once

#include <cmath>
#include <tuple>

constexpr float map(const float value, const float x_min, const float x_max, const float y_min, const float y_max) {
    if (value < x_min)
        return y_min;
    if (value > x_max)
        return y_max;
    return (value - x_min) * (y_max - y_min) / (x_max - x_min) + y_min;
}

inline std::tuple<float, float> rotate(const float x, const float y, const float angle) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return {c * x - s * y, s * x + c * y};
}
