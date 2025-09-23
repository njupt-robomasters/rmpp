#pragma once

#include <tuple>

float solve_alpha(float fc, float fs);

float lowpass_filter(float x, float next, float alpha);

float clamp(float x, float max);

float clamp(float x, float min, float max);

template <typename T1, typename T2>
std::tuple<T1, T1> rotate(const T1 x, const T1 y, const T2 angle) {
    const float cos = arm_cos_f32(angle);
    const float sin = arm_sin_f32(angle);
    const T1 rotate_x = cos * x - sin * y;
    const T1 rotate_y = sin * x + cos * y;
    return {rotate_x, rotate_y};
}
