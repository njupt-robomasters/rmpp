#pragma once

#include <tuple>
#include "UnitFloat.hpp"
#include "arm_math.h"

namespace unit {
    template <const Unit &T = default_unit>
    class Angle : public UnitFloat<T> {
    public:
        // 无参构造，使用父类构造函数
        constexpr Angle() : UnitFloat<T>() {}

        // 用unit构造，使用父类构造函数
        constexpr Angle(const Unit& unit) : UnitFloat<T>(unit) {}

        // 拷贝构造
        template <const Unit&T2>
        constexpr Angle(const UnitFloat<T2>& other) : UnitFloat<T>(other) {
            normalize();
        }

        // 赋值运算
        template <const Unit&T2>
        constexpr Angle& operator=(const UnitFloat<T2>& other) {
            UnitFloat<T>::operator=(other);
            normalize();
            return *this;
        }

        // 特化版本，修复某个bug
        constexpr Angle& operator=(const Angle& other) {
            UnitFloat<T>::operator=(other);
            normalize();
            return *this;
        }

        // 移动赋值
        template <const Unit&T2>
        constexpr Angle& operator=(UnitFloat<T2>&& other) {
            UnitFloat<T>::operator=(other);
            normalize();
            return *this;
        }

        // +=运算
        template <const Unit&T2>
        constexpr Angle& operator+=(const UnitFloat<T2>& other) {
            UnitFloat<T>::operator+=(other);
            normalize();
            return *this;
        }

        // -=运算
        template <const Unit&T2>
        constexpr Angle& operator-=(const UnitFloat<T2>& other) {
            UnitFloat<T>::operator-=(other);
            normalize();
            return *this;
        }

        // *=运算
        template <const Unit&T2>
        constexpr Angle& operator*=(const UnitFloat<T2>& other) {
            UnitFloat<T>::operator*=(other);
            normalize();
            return *this;
        }

        // 乘法比例运算
        constexpr Angle& operator*=(const float value) {
            UnitFloat<T>::operator*=(value);
            normalize();
            return *this;
        }

        // /=运算
        template <const Unit&T2>
        constexpr Angle& operator/=(const UnitFloat<T2>& other) {
            UnitFloat<T>::operator/=(other);
            normalize();
            return *this;
        }

        // 除法比例运算
        constexpr Angle& operator/=(const float value) {
            UnitFloat<T>::operator/=(value);
            normalize();
            return *this;
        }

    private:
        // 角度归一化
        constexpr void normalize() {
            float angle = this->toFloat();
            while (angle <= -PI)
                angle += 2 * PI;
            while (angle > PI)
                angle -= 2 * PI;
            this->value = angle / this->unit.scale;
        }
    };

    // sin
    constexpr UnitFloat<> sin(const Angle<>& angle) {
        UnitFloat ret(default_unit); // 返回值无单位
        ret.value = std::sin(angle.toFloat());
        return ret;
    }

    // cos
    constexpr UnitFloat<> cos(const Angle<>& angle) {
        UnitFloat ret(default_unit); // 返回值无单位
        ret.value = std::cos(angle.toFloat());
        return ret;
    }

    // tan
    constexpr UnitFloat<> tan(const Angle<>& angle) {
        UnitFloat ret(default_unit); // 返回值无单位
        ret.value = std::tan(angle.toFloat());
        return ret;
    }

    // atan2
    constexpr Angle<> atan2(const UnitFloat<>& y, const UnitFloat<>& x) {
        Angle ret(default_unit); // 返回值无单位
        ret.value = std::atan2(y.toFloat(), x.toFloat());
        return ret;
    }

    // 旋转
    inline std::tuple<UnitFloat<>, UnitFloat<>> rotate(const UnitFloat<>& x, const UnitFloat<>& y, const Angle<>& angle) {
        const float cos = std::cos(angle.toFloat());
        const float sin = std::sin(angle.toFloat());
        const UnitFloat rotate_x = cos * x - sin * y;
        const UnitFloat rotate_y = sin * x + cos * y;
        return {rotate_x, rotate_y};
    }
}