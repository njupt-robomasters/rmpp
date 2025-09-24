#pragma once

#include <tuple>
#include "arm_math.h"

float solve_alpha(float fc, float fs);

float lowpass_filter(float x, float next, float alpha);

float clamp(float x, float max);

float clamp(float x, float min, float max);

template <typename T>
std::tuple<T, T> rotate(const T x, const T y, const float angle) {
    const float cos = arm_cos_f32(angle);
    const float sin = arm_sin_f32(angle);
    const T rotate_x = cos * x - sin * y;
    const T rotate_y = sin * x + cos * y;
    return {rotate_x, rotate_y};
}
