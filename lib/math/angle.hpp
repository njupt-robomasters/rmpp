#pragma once

#include "unit.hpp"
#include <limits>

class Angle {
  public:
    constexpr Angle()
        : radians_(0.0f) {}

    // 角度统一使用弧度，构造时归一化
    constexpr Angle(float radians)
        : radians_(normalize(radians)) {}

    constexpr operator float() const {
        return radians_;
    }

    constexpr Angle& operator=(float radians) {
        radians_ = normalize(radians);
        return *this;
    }

    constexpr Angle& operator+=(float radians) {
        radians_ = normalize(radians_ + radians);
        return *this;
    }

    constexpr Angle& operator-=(float radians) {
        radians_ = normalize(radians_ - radians);
        return *this;
    }

    constexpr Angle& operator*=(float factor) {
        radians_ = normalize(radians_ * factor);
        return *this;
    }

    constexpr Angle& operator/=(float factor) {
        radians_ = normalize(radians_ / factor);
        return *this;
    }

  private:
    static constexpr float normalize(float radians) {
        constexpr float max = std::numeric_limits<float>::max();
        if (radians != radians || radians > max || radians < -max)
            return radians;
        if (radians >= -pi && radians < pi)
            return radians;

        // 二进制取余支持 C++14 常量求值，迭代次数受浮点指数范围限制
        // 逐次减去一圈时，大数输入可能因浮点精度限制而无法继续减小
        float result = radians < 0.0f ? -radians : radians;
        float period = rev;
        while (period <= result / 2.0f)
            period *= 2.0f;
        while (period >= rev) {
            if (result >= period)
                result -= period;
            period *= 0.5f;
        }
        if (radians < 0.0f)
            result = -result;
        if (result >= pi)
            result -= rev;
        if (result < -pi)
            result += rev;
        return result;
    }

    float radians_;
};
