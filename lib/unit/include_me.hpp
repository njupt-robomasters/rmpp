#pragma once

#include "unit.hpp"
#include "unit_float.hpp"
#include "angle.hpp"

using unit::Unit;
using unit::UnitFloat;
using unit::Angle;

using unit::default_unit;

// 简单单位
// 时间
inline constexpr Unit s("s", 1.0f);
inline constexpr Unit min("min", 60.0f);
inline constexpr Unit ms("ms", 0.001f);
inline constexpr Unit us("us", 1.0e-6f);

// 长度
inline constexpr Unit m("m", 1.0f);
inline constexpr Unit cm("cm", 0.01f);
inline constexpr Unit mm("mm", 0.001f);

// 角度
inline constexpr Unit rad("rad", 1.0f);        // 弧度
inline constexpr Unit deg("deg", PI / 180.0f); // 角度
inline constexpr Unit rev("rev", 2 * PI);      // 圈

// 力
inline constexpr Unit N("N", 1.0f);   // 力
inline constexpr Unit Nm("Nm", 1.0f); // 扭矩

// 电学
inline constexpr Unit V("V", 1.0f);     // 电压
inline constexpr Unit A("A", 1.0f);     // 电流
inline constexpr Unit W("W", 1.0f);     // 功率
inline constexpr Unit Ohm("Ohm", 1.0f); // 电阻

// 比例
inline constexpr Unit ratio("ratio", 1.0f); // 比例
inline constexpr Unit pct("%", 0.01f);      // 百分比

// 其他
inline constexpr Unit Hz("Hz", 1.0f); // 频率
inline constexpr Unit C("C", 1.0f);   // 摄氏度

// 组合单位
// 速度
inline constexpr Unit m_s = m / s;

// 角速度
inline constexpr Unit rad_s = rad / s;
inline constexpr Unit deg_s = deg / s;
inline constexpr Unit rpm("rpm", (rev / min).m_scale);
inline constexpr Unit rps("rps", (rev / s).m_scale);

// 加速度
inline constexpr Unit m_ss = m / s / s;
inline constexpr Unit deg_ss = deg / s / s;

// 扭矩系数
inline constexpr Unit Nm_A = Nm / A;