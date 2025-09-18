#pragma once

#include <string_view>
#include "arm_math.h"

class U {
public:
    const std::string_view m_str;
    const float m_scale;

    constexpr U(const std::string_view str, const float scale) : m_str(str), m_scale(scale) {}

    constexpr operator float() const {
        return m_scale;
    }
};

// 长度
inline constexpr U m("m", 1.0f);
inline constexpr U cm("cm", 0.01f);
inline constexpr U mm("mm", 0.001f);

// 时间
inline constexpr U s("s", 1.0f);
inline constexpr U ms("ms", 0.001f);

// 速度
inline constexpr U m_s("m/s", 1.0f);

// 角度
inline constexpr U rad("rad", 1.0f);
inline constexpr U deg("deg", 1 / 180.0f * PI);

// 角速度
inline constexpr U rad_s("rad/s", 1.0f);
inline constexpr U deg_s("deg/s", 1 / 180.0f * PI);
inline constexpr U rpm("rpm", 1 / 60.0f * (2 * PI));
inline constexpr U rps("rps", 2 * PI);

// 电压、电流、功率
inline constexpr U V("V", 1.0f);
inline constexpr U A("A", 1.0f);
inline constexpr U W("W", 1.0f);

// 其他
inline constexpr U no_unit("no_unit", 1.0f); // 无单位
inline constexpr U percentage("%", 0.01f); // 百分比
inline constexpr U Hz("Hz", 1.0f); // 频率
inline constexpr U C("℃", 1.0f); // ℃
inline constexpr U Nm("Nm", 1.0f); // 扭矩


template <const U& u = no_unit>
class Unit {
protected:
    float m_value = 0.0f;
    float m_scale;
    std::string_view m_str;

public:
    // 无参构造
    constexpr Unit() : m_scale(u.m_scale), m_str(u.m_str) {}

    // 从float隐式转换构造
    constexpr Unit(const float value) : Unit() {
        m_value = value / m_scale;
    }

    // 从float隐式转换赋值
    constexpr Unit& operator=(const float value) {
        m_value = value / m_scale;
        return *this;
    }

    constexpr operator float() const {
        return m_value * m_scale;
    }

    constexpr float operator+(const Unit& other) const {
        return (float)(*this) + (float)other;
    }

    constexpr float operator-(const Unit& other) const {
        return (float)(*this) - (float)other;
    }

    constexpr float operator*(const Unit& other) const {
        return (float)(*this) * (float)other;
    }

    constexpr float operator/(const Unit& other) const {
        return (float)(*this) / (float)other;
    }

    constexpr float operator+(const float other) const {
        return (float)(*this) + other;
    }

    constexpr float operator-(const float other) const {
        return (float)(*this) - other;
    }

    constexpr float operator*(const float other) const {
        return (float)(*this) * other;
    }

    constexpr float operator/(const float other) const {
        return (float)(*this) / other;
    }

    constexpr float get(const U& u_) {
        return (float)(*this) / u_.m_scale;
    }
};

template <const U& u = no_unit>
class Angle : public Unit<u> {
private:
    constexpr static float normalize(float angle) {
        while (angle < -PI) angle += 2 * PI;
        while (angle >= PI) angle -= 2 * PI;
        return angle;
    }

public:
    // 无参构造
    constexpr Angle() : Unit<u>() {}

    // 从float隐式转换构造
    constexpr Angle(const float value) {
        this->m_value = normalize(value) / this->m_scale;
    }

    // 从float隐式转换赋值
    constexpr Angle& operator=(const float value) {
        this->m_value = normalize(value) / this->m_scale;
        return *this;
    }
};
