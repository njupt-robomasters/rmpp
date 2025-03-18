#include "utils.hpp"
#include <cmath>

float signed_sqrt(const float x) {
    return x < 0 ? -sqrtf(-x) : sqrtf(x);
}

float solve_alpha(const float fc, const float fs) {
    constexpr auto m_2_pi = static_cast<float>(M_2_PI);
    const float alpha = m_2_pi * fc / (fs + m_2_pi * fc);
    return alpha;
}

void lowpass_filter(float &x, const float next, const float alpha) {
    x = x * (1 - alpha) + next * alpha;
}

float norm_angle(const float angle) {
    float result = fmodf(angle, 360.0f);
    if (result < 0) {
        result += 360.0f;
    }
    return result;
}

float calc_angle_err(float ref, float measure) {
    ref = norm_angle(ref);
    measure = norm_angle(measure);

    const float err = fmodf(ref - measure + 540.0f, 360.0f) - 180.0f; // 最短路径误差

    return err;
}

void clamp(float &x, const float max) {
    if (x > max) {
        x = max;
    }
    if (x < -max) {
        x = -max;
    }
}

void clamp(float &x, const float min, const float max) {
    if (x > max) {
        x = max;
    }
    if (x < min) {
        x = min;
    }
}
