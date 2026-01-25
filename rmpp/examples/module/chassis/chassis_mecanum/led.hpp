#pragma once

#include "app/LED.hpp"

inline LED::config_t led_config = {
    .period = 1 * s,
    .saturation = 1 * ratio,
    .brightness = 1 * ratio,
};
inline LED led(led_config);