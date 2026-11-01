#pragma once

#include "unit.hpp"

// 一阶 RC 低通：fc 为截止频率（Hz），dt 为距上次输出的间隔（s）；fc 为 0 时直通
template <typename T>
T lowpass(const T last, const T input, const float fc, const float dt) {
    if (fc == 0)
        return input;
    const float rc = 1 / (2 * pi * fc);
    const float alpha = dt / (rc + dt);
    return last + alpha * (input - last);
}
