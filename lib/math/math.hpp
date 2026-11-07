#pragma once

constexpr float map(const float value, const float x_min, const float x_max, const float y_min, const float y_max) {
    if (value < x_min)
        return y_min;
    if (value > x_max)
        return y_max;
    return (value - x_min) * (y_max - y_min) / (x_max - x_min) + y_min;
}
