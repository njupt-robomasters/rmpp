#pragma once

#include "unit.hpp"
#include <cmath>

class Angle {
  public:
    constexpr Angle() : radians_(0.0f) {}

    // 角度统一使用弧度，构造时归一化
    constexpr Angle(const float radians) : radians_(normalize(radians)) {}

    constexpr operator float() const {
        return radians_;
    }

    constexpr Angle& operator=(const float radians) {
        radians_ = normalize(radians);
        return *this;
    }

    constexpr Angle& operator+=(const float radians) {
        radians_ = normalize(radians_ + radians);
        return *this;
    }

    constexpr Angle& operator-=(const float radians) {
        radians_ = normalize(radians_ - radians);
        return *this;
    }

    constexpr Angle& operator*=(const float factor) {
        radians_ = normalize(radians_ * factor);
        return *this;
    }

    constexpr Angle& operator/=(const float factor) {
        radians_ = normalize(radians_ / factor);
        return *this;
    }

  private:
    float radians_;

    static constexpr float normalize(const float radians) {
        if (!std::isfinite(radians))
            return radians;
        if (radians >= -pi && radians < pi)
            return radians;

        float result = std::fmod(radians, rev);
        if (result >= pi)
            result -= rev;
        if (result < -pi)
            result += rev;
        return result;
    }
};
