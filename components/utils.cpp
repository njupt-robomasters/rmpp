#include "utils.hpp"
#include "arm_math.h"

float signed_sqrt(const float x) {
    return x < 0 ? -sqrtf(-x) : sqrtf(x);
}

float solve_alpha(const float fc, const float fs) {
    const float alpha = 2 * PI * fc / (fs + 2 * PI * fc);
    return alpha;
}

float lowpass_filter(float x, const float next, const float alpha) {
    x = x * (1 - alpha) + next * alpha;
    return x;
}

float norm_angle(const float degree) {
    float result = fmodf(degree, 360.0f);
    if (result < 0) {
        result += 360.0f;
    }
    return result;
}

float calc_angle_err(float ref_degree, float measure_degree) {
    ref_degree = norm_angle(ref_degree);
    measure_degree = norm_angle(measure_degree);

    const float err = fmodf(ref_degree - measure_degree + 540.0f, 360.0f) - 180.0f; // 最短路径误差

    return err;
}

float clamp(float x, const float max) {
    if (x > max) {
        x = max;
    }
    if (x < -max) {
        x = -max;
    }
    return x;
}

float clamp(float x, const float min, const float max) {
    if (x > max) {
        x = max;
    }
    if (x < min) {
        x = min;
    }
    return x;
}

std::tuple<float, float> rotate(const float x, const float y, const float angle) {
    const float rad = angle / 180.0f * PI;
    const float cos = arm_cos_f32(rad);
    const float sin = arm_sin_f32(rad);
    const float rotate_x = cos * x - sin * y;
    const float rotate_y = sin * x + cos * y;
    return {rotate_x, rotate_y};
}
