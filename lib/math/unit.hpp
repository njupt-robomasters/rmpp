#pragma once

inline constexpr float pi = 3.14159265358979323846f;

// 换算到内部国际单位制表示的系数
inline constexpr float ms = 1.0e-3f;
inline constexpr float us = 1.0e-6f;

inline constexpr float cm = 1.0e-2f;
inline constexpr float mm = 1.0e-3f;

inline constexpr float deg = pi / 180.0f;
inline constexpr float rev = 2.0f * pi;

inline constexpr float mA = 1.0e-3f;

inline constexpr float pct = 1.0e-2f;

inline constexpr float deg_s = deg; // 度每秒 -> rad/s
inline constexpr float rpm = rev / 60.0f;
inline constexpr float rps = rev;    // 转每秒 -> rad/s
inline constexpr float deg_ss = deg; // 度每二次方秒 -> rad/s^2
