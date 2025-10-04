#pragma once

#include <string_view>
#include "arm_math.h"

class Unit {
public:
    const float m_scale;
    const std::string_view m_str;

    constexpr Unit(const std::string_view str, const float scale) : m_str(str), m_scale(scale) {}

    constexpr operator float() const {
        return m_scale;
    }
};

// 长度
inline constexpr Unit m("m", 1.0f);
inline constexpr Unit cm("cm", 0.01f);
inline constexpr Unit mm("mm", 0.001f);

// 时间
inline constexpr Unit s("s", 1.0f);
inline constexpr Unit ms("ms", 0.001f);

// 速度
inline constexpr Unit m_s("m/s", 1.0f);

// 加速度
inline constexpr Unit m_ss("m/s^2", 1.0f);

// 角度
inline constexpr Unit rad("rad", 1.0f);
inline constexpr Unit deg("deg", 1 / 180.0f * PI);

// 角速度
inline constexpr Unit rad_s("rad/s", 1.0f);
inline constexpr Unit deg_s("deg/s", 1 / 180.0f * PI);
inline constexpr Unit rpm("rpm", 1 / 60.0f * (2 * PI));
inline constexpr Unit rps("rps", 2 * PI);

// 电压、电流、功率、扭矩
inline constexpr Unit V("V", 1.0f);
inline constexpr Unit A("A", 1.0f);
inline constexpr Unit W("W", 1.0f);
inline constexpr Unit Nm("Nm", 1.0f);

// pid单位
inline constexpr Unit A_deg("A/deg", A / deg);
inline constexpr Unit Nm_deg("Nm/deg", Nm / deg);
inline constexpr Unit A_rpm("A/rpm", A / rpm);

// 其他
inline constexpr Unit no_unit("", 1.0f); // 无单位
inline constexpr Unit percentage("%", 0.01f); // 百分比
inline constexpr Unit Hz("Hz", 1.0f); // 频率
inline constexpr Unit C("℃", 1.0f); // ℃


template <const Unit& T = no_unit>
class UnitFloat {
public:
    float m_value = 0.0f;
    float m_scale;
    std::string_view m_str;

    // 无参构造
    constexpr UnitFloat() : m_scale(T.m_scale), m_str(T.m_str) {}

    // 从float构造
    constexpr UnitFloat(const float value) : UnitFloat() {
        m_value = value / m_scale;
    }

    // 从另一种模板实例构造
    template <const Unit& U>
    constexpr UnitFloat(const UnitFloat<U>& other) {
        if (T.m_str.empty()) { // 自己的模板无单位，用other的单位
            m_scale = other.m_scale;
            m_str = other.m_str;
        } else { // 自己的模板有单位，用自己的单位
            m_scale = T.m_scale;
            m_str = T.m_str;
        }
        m_value = (float)other / m_scale;
    }

    // 从float赋值
    constexpr UnitFloat& operator=(const float value) {
        m_value = value / m_scale;
        return *this;
    }

    // 从另一种模板实例赋值
    template <const Unit& U>
    constexpr UnitFloat& operator=(const UnitFloat<U>& other) {
        if (m_str.empty()) { // 自己无单位，用other的单位
            m_scale = other.m_scale;
            m_str = other.m_str;
        }
        m_value = (float)other / m_scale;
        return *this;
    }

    // +=运算
    constexpr UnitFloat& operator+=(const float other) {
        m_value += other / m_scale;
        return *this;
    }

    // -=运算
    constexpr UnitFloat& operator-=(const float other) {
        m_value -= other / m_scale;
        return *this;
    }

    // 转换到float
    constexpr operator float() const {
        return m_value * m_scale;
    }

    // 取相反数
    constexpr UnitFloat operator-() const {
        UnitFloat ret;
        ret.m_value = -m_value;
        ret.m_scale = m_scale;
        ret.m_str = m_str;
        return ret;
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

    template <const Unit& U>
    constexpr float operator+(const UnitFloat<U>& other) const {
        return (float)(*this) + (float)other;
    }

    template <const Unit& U>
    constexpr float operator-(const UnitFloat<U>& other) const {
        return (float)(*this) - (float)other;
    }

    template <const Unit& U>
    constexpr float operator*(const UnitFloat<U>& other) const {
        return (float)(*this) * (float)other;
    }

    template <const Unit& U>
    constexpr float operator/(const UnitFloat<U>& other) const {
        return (float)(*this) / (float)other;
    }

    template <const Unit& U>
    constexpr bool operator<(const UnitFloat<U>& other) const {
        return (float)(*this) < (float)other;
    }

    template <const Unit& U>
    constexpr bool operator>(const UnitFloat<U>& other) const {
        return (float)(*this) > (float)other;
    }

    constexpr float get(const Unit& unit) {
        return (float)(*this) / unit.m_scale;
    }
};

template <const Unit& T = no_unit>
class Angle : public UnitFloat<T> {
private:
    constexpr static float normalize(float angle) {
        while (angle < -PI) angle += 2 * PI;
        while (angle >= PI) angle -= 2 * PI;
        return angle;
    }

public:
    // 无参构造，使用父类构造函数
    constexpr Angle() : UnitFloat<T>() {}

    // 从float构造
    constexpr Angle(const float value) {
        this->m_value = normalize(value) / this->m_scale;
    }

    // 从另一种模板实例构造
    template <const Unit& U>
    constexpr Angle(const UnitFloat<U>& other) {
        if (T.m_str.empty()) { // 自己的模板无单位，用other的单位
            this->m_scale = other.m_scale;
            this->m_str = other.m_str;
        } else { // 自己的模板有单位，用自己的单位
            this->m_scale = T.m_scale;
            this->m_str = T.m_str;
        }
        this->m_value = normalize((float)other) / this->m_scale;
    }

    // 从float赋值
    constexpr Angle& operator=(const float value) {
        this->m_value = normalize(value) / this->m_scale;
        return *this;
    }

    // 从另一种模板实例赋值
    template <const Unit& U>
    constexpr Angle& operator=(const UnitFloat<U>& other) {
        if (this->m_str.empty()) { // 自己无单位，用other的单位
            this->m_scale = other.m_scale;
            this->m_str = other.m_str;
        }
        this->m_value = normalize((float)other) / this->m_scale;
        return *this;
    }

    // +=运算
    constexpr Angle& operator+=(const float other) {
        this->m_value = normalize((float)(*this) + other) / this->m_scale;
        return *this;
    }
};
