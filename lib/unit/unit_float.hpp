#pragma once

#include <cmath>
#include "unit.hpp"

namespace unit {
    template <const Unit &T = default_unit>
    class UnitFloat {
    public:
        float m_value = 0.0f;
        Unit m_unit; // 单位，包括str和scale

        // 无参构造
        constexpr UnitFloat() : m_unit(T) {}

        // 指定unit构造
        constexpr UnitFloat(const Unit& unit) : m_unit(unit) {}

        // 拷贝构造
        template <const Unit&T2>
        constexpr UnitFloat(const UnitFloat<T2>& other) : m_unit(T) {
            m_unit = other.m_unit;
            m_value = other.toFloat(m_unit);
        }

        // 赋值运算
        template <const Unit&T2>
        constexpr UnitFloat& operator=(const UnitFloat<T2>& other) {
            m_unit = other.m_unit;
            m_value = other.toFloat(m_unit);
            return *this;
        }

        // 取正数
        constexpr UnitFloat operator+() const {
            return *this;
        }

        // 取相反数
        constexpr UnitFloat operator-() const {
            UnitFloat ret = *this;
            ret.m_value = -m_value;
            return ret;
        }

        // 四则运算，加
        template <const Unit&T2>
        constexpr UnitFloat operator+(const UnitFloat<T2>& other) const {
            UnitFloat ret(default_unit);
            ret.m_unit = m_unit + other.m_unit;
            ret.m_value = toFloat(ret.m_unit) + other.toFloat(ret.m_unit);
            return ret;
        }

        template <const Unit&T2>
        constexpr UnitFloat& operator+=(const UnitFloat<T2>& other) {
            *this = *this + other;
            return *this;
        }

        // 四则运算，减
        template <const Unit&T2>
        constexpr UnitFloat operator-(const UnitFloat<T2>& other) const {
            UnitFloat ret(default_unit);
            ret.m_unit = m_unit - other.m_unit;
            ret.m_value = toFloat(ret.m_unit) - other.toFloat(ret.m_unit);
            return ret;
        }

        template <const Unit&T2>
        constexpr UnitFloat& operator-=(const UnitFloat<T2>& other) {
            *this = *this - other;
            return *this;
        }

        // 四则运算，乘
        template <const Unit&T2>
        constexpr UnitFloat operator*(const UnitFloat<T2>& other) const {
            UnitFloat ret(default_unit);
            ret.m_unit = m_unit * other.m_unit;
            ret.m_value = m_value * other.m_value;
            return ret;
        }

        template <const Unit&T2>
        constexpr UnitFloat& operator*=(const UnitFloat<T2>& other) {
            *this = *this * other;
            return *this;
        }

        constexpr UnitFloat operator*(const float value) const { // 乘法比例运算
            UnitFloat ret = *this;                               // 返回值和当前同单位
            ret.m_value *= value;
            return ret;
        }

        constexpr UnitFloat& operator*=(const float value) { // 乘法比例运算
            m_value *= value;
            return *this;
        }

        // 四则运算，除
        template <const Unit&T2>
        constexpr UnitFloat operator/(const UnitFloat<T2>& other) const {
            UnitFloat ret = *this;
            ret.m_unit /= other.m_unit; // 返回值单位
            ret.m_value /= other.m_value;
            return ret;
        }

        template <const Unit&T2>
        constexpr UnitFloat& operator/=(const UnitFloat<T2>& other) {
            *this = *this / other;
            return *this;
        }

        constexpr UnitFloat operator/(const float value) const { // 除法比例运算
            UnitFloat ret = *this;                               // 返回值和当前同单位
            ret.m_value /= value;
            return ret;
        }

        constexpr UnitFloat& operator/=(const float value) { // 除法比例运算
            m_value /= value;
            return *this;
        }

        // 比较运算，等于
        template <const Unit&T2>
        constexpr bool operator==(const UnitFloat<T2>& other) const {
            return toFloat() == other.toFloat();
        }

        constexpr bool operator==(const float value) const {
            return toFloat() == value;
        }

        // 比较运算，不等于
        template <const Unit&T2>
        constexpr bool operator!=(const UnitFloat<T2>& other) const {
            return toFloat() != other.toFloat();
        }

        constexpr bool operator!=(const float value) const {
            return toFloat() != value;
        }

        // 比较运算，小于
        template <const Unit&T2>
        constexpr bool operator<(const UnitFloat<T2>& other) const {
            return toFloat() < other.toFloat();
        }

        constexpr bool operator<(const float value) const {
            return toFloat() < value;
        }

        // 比较运算，小于等于
        template <const Unit&T2>
        constexpr bool operator<=(const UnitFloat<T2>& other) const {
            return toFloat() <= other.toFloat();
        }

        constexpr bool operator<=(const float value) const {
            return toFloat() <= value;
        }

        // 比较运算，大于
        template <const Unit&T2>
        constexpr bool operator>(const UnitFloat<T2>& other) const {
            return toFloat() > other.toFloat();
        }

        constexpr bool operator>(const float value) const {
            return toFloat() > value;
        }

        // 比较运算，大于等于
        template <const Unit&T2>
        constexpr bool operator>=(const UnitFloat<T2>& other) const {
            return toFloat() >= other.toFloat();
        }

        constexpr bool operator>=(const float value) const {
            return toFloat() >= value;
        }

        // 转换到浮点数
        constexpr float toFloat() const {
            return m_value * m_unit.m_scale;
        }

        // 转换到浮点数，给定单位
        constexpr float toFloat(const Unit& unit) const {
            return m_value * m_unit.m_scale / unit.m_scale;
        }

        // 更新单位
        constexpr void UpdateUnit(const Unit& unit) {
            // 下面两行顺序不能换
            m_value = m_value * m_unit.m_scale / unit.m_scale;
            m_unit = unit;
        }
    };

    // 后置实现 Unit * float，因为当时UnitFloat还未定义
    constexpr auto Unit::operator*(const float value) const {
        UnitFloat ret(default_unit);
        ret.m_value = value;
        ret.m_unit = *this;
        return ret;
    }

    // 重载全局运算符
    // float * Unit
    constexpr UnitFloat<> operator*(const float value, const Unit& unit) {
        return unit * value;
    }

    // float * UnitFloat
    constexpr UnitFloat<> operator*(const float value, const UnitFloat<>& unit_float) {
        return unit_float * value;
    }

    // float / UnitFloat
    constexpr UnitFloat<> operator/(const float value, const UnitFloat<>& unit_float) {
        UnitFloat ret(default_unit);
        ret.m_unit = default_unit / unit_float.m_unit;
        ret.m_value = value / unit_float.m_value;
        return ret;
    }

    // 取绝对值
    constexpr UnitFloat<> abs(const UnitFloat<>& x) {
        if (x >= 0) {
            return x;
        } else {
            return -x;
        }
    }

    // 平方
    constexpr UnitFloat<> square(const UnitFloat<>& x) {
        UnitFloat ret = x * x;
        return ret;
    }

    // 开方
    constexpr UnitFloat<> sqrt(const UnitFloat<>& x) {
        UnitFloat ret(default_unit); // 返回值无单位
        ret.m_value = std::sqrt(x.toFloat());
        return ret;
    }

    // 钳位
    constexpr UnitFloat<> clamp(const UnitFloat<>& x, const UnitFloat<>& max) {
        if (x > max) {
            return max;
        } else if (x < -max) {
            return -max;
        } else {
            return x;
        }
    }

    constexpr UnitFloat<> clamp(const UnitFloat<>& x, const UnitFloat<>& min, const UnitFloat<>& max) {
        if (x > max) {
            return max;
        } else if (x < min) {
            return min;
        } else {
            return x;
        }
    }
}