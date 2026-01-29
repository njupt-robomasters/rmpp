#pragma once

#include <cmath>
#include "Unit.hpp"

namespace unit {
    template <const Unit &T = default_unit>
    class UnitFloat {
    public:
        float value = 0; // 值
        Unit unit;       // 单位

        // 无参构造，用模板的单位
        constexpr UnitFloat() : unit(T) {}

        // 用unit构造
        constexpr UnitFloat(const Unit& unit) : unit(unit) {}

        // 拷贝构造
        template <const Unit&T2>
        constexpr UnitFloat(const UnitFloat<T2>& other) : unit(T) {
            this->unit = other.unit; // 模板无单位才会更新
            this->value = other.toFloat(this->unit);
        }

        // 赋值运算
        template <const Unit&T2>
        constexpr UnitFloat& operator=(const UnitFloat<T2>& other) {
            this->unit = other.unit; // 模板无单位才会更新
            this->value = other.toFloat(this->unit);
            return *this;
        }

        // 特化版本，修复某个bug
        constexpr UnitFloat& operator=(const UnitFloat& other) {
            this->unit = other.unit; // 模板无单位才会更新
            this->value = other.toFloat(this->unit);
            return *this;
        }

        // 移动赋值
        template <const Unit&T2>
        constexpr UnitFloat& operator=(UnitFloat<T2>&& other) {
            this->unit = other.unit; // 模板无单位才会更新
            this->value = other.toFloat(this->unit);
            return *this;
        }

        // 取正数
        constexpr UnitFloat operator+() const {
            UnitFloat ret = *this;
            return ret;
        }

        // 取相反数
        constexpr UnitFloat operator-() const {
            UnitFloat ret = *this;
            ret.value = -value;
            return ret;
        }

        // 四则运算，加
        template <const Unit&T2>
        constexpr UnitFloat operator+(const UnitFloat<T2>& other) const {
            UnitFloat ret(default_unit);
            ret.unit = this->unit + other.unit; // 会优先用左操作数的单位
            ret.value = toFloat(ret.unit) + other.toFloat(ret.unit);
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
            ret.unit = this->unit - other.unit; // 会优先用左操作数的单位
            ret.value = toFloat(ret.unit) - other.toFloat(ret.unit);
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
            ret.unit = this->unit * other.unit;
            ret.value = this->value * other.value;
            return ret;
        }

        template <const Unit&T2>
        constexpr UnitFloat& operator*=(const UnitFloat<T2>& other) {
            *this = *this * other;
            return *this;
        }

        constexpr UnitFloat operator*(const float value) const { // 乘法比例运算
            UnitFloat ret = *this;
            ret.value *= value;
            return ret;
        }

        constexpr UnitFloat& operator*=(const float value) { // 乘法比例运算
            this->value *= value;
            return *this;
        }

        // 四则运算，除
        template <const Unit&T2>
        constexpr UnitFloat operator/(const UnitFloat<T2>& other) const {
            UnitFloat ret(default_unit);
            ret.unit = this->unit / other.unit;
            ret.value = this->value / other.value;
            return ret;
        }

        template <const Unit&T2>
        constexpr UnitFloat& operator/=(const UnitFloat<T2>& other) {
            *this = *this / other;
            return *this;
        }

        constexpr UnitFloat operator/(const float value) const { // 除法比例运算
            UnitFloat ret = *this;
            ret.value /= value;
            return ret;
        }

        constexpr UnitFloat& operator/=(const float value) { // 除法比例运算
            this->value /= value;
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
            return this->value * this->unit.scale;
        }

        // 转换到浮点数，给定单位
        constexpr float toFloat(const Unit& unit) const {
            return this->value * this->unit.scale / unit.scale;
        }

        // 更新单位
        // constexpr void UpdateUnit(const Unit& unit) {
        //     // 下面两行顺序不能换
        //     this->value = this->value * this->unit.scale / unit.scale;
        //     this->unit = unit;
        // }
    };

    // 后置实现 Unit * float，因为当时返回值UnitFloat还未定义
    constexpr auto Unit::operator*(const float value) const {
        UnitFloat ret(default_unit);
        ret.value = value;
        ret.unit = *this;
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
        ret.unit = default_unit / unit_float.unit;
        ret.value = value / unit_float.value;
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
        ret.value = std::sqrt(x.toFloat());
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