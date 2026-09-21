#pragma once

namespace math {

    inline constexpr float pi = 3.14159265358979323846f;

    // Conversion factors into the internal SI representation.
    inline constexpr float ms = 1.0e-3f;
    inline constexpr float us = 1.0e-6f;

    inline constexpr float cm = 1.0e-2f;
    inline constexpr float mm = 1.0e-3f;

    inline constexpr float deg = pi / 180.0f;
    inline constexpr float rev = 2.0f * pi;

    inline constexpr float mA = 1.0e-3f;

    inline constexpr float pct = 1.0e-2f;

    inline constexpr float deg_s = deg; // degrees/s -> rad/s
    inline constexpr float rpm = rev / 60.0f;
    inline constexpr float rps = rev;    // revolutions/s -> rad/s
    inline constexpr float deg_ss = deg; // degrees/s^2 -> rad/s^2

} // namespace math
