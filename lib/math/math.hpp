#pragma once

#include <cmath>
#include <tuple>

namespace math {

constexpr float square(float value) { return value * value; }

constexpr float clamp(float value, float max_value) {
    if (value > max_value)
        return max_value;
    if (value < -max_value)
        return -max_value;
    return value;
}

constexpr float clamp(float value, float min_value, float max_value) {
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

constexpr float map(float value, float x_min, float x_max, float y_min, float y_max) {
    if (value < x_min)
        return y_min;
    if (value > x_max)
        return y_max;
    return (value - x_min) * (y_max - y_min) / (x_max - x_min) + y_min;
}

inline std::tuple<float, float> rotate(float x, float y, float angle) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return {c * x - s * y, s * x + c * y};
}

} // namespace math
