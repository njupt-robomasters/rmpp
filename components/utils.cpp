#include "utils.hpp"
#include <algorithm>

float solve_alpha(const float fc, const float fs) {
    const float alpha = 2 * PI * fc / (fs + 2 * PI * fc);
    return alpha;
}

float lowpass_filter(float x, const float next, const float alpha) {
    x = x * (1 - alpha) + next * alpha;
    return x;
}

float clamp(float x, const float max) {
    return std::clamp(x, -max, max);
}

float clamp(float x, const float min, const float max) {
    return std::clamp(x, min, max);
}
