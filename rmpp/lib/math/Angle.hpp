#pragma once

#include <limits>
#include "Conversions.hpp"

namespace math {

    class Angle {
    public:
        constexpr Angle() :
            radians_(0.0f) {}

        // Angles are always represented in radians and normalized at construction.
        constexpr Angle(float radians) :
            radians_(normalize(radians)) {}

        constexpr operator float() const { return radians_; }

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

            // Binary remainder is constexpr in C++14 and bounded by the float exponent range.
            // Subtracting one turn repeatedly can stop making progress for large inputs.
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

} // namespace math
