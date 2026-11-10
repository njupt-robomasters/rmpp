#pragma once

#include "eigen.hpp"
#include "unit.hpp"
#include <algorithm>
#include <cmath>
#include <tuple>

class Angle {
  public:
    constexpr Angle()
        : radians_(0.0f) {}

    // 角度统一使用弧度，构造时归一化
    constexpr Angle(const float radians)
        : radians_(normalize(radians)) {}

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

// 平面矢量 (x, y) 逆时针旋转 angle；坐标系换算时，新系相对旧系转了 θ 则传 -θ
inline std::tuple<float, float> rotate(const float x, const float y, const float angle) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return {c * x - s * y, s * x + c * y};
}

// 绕 Z 轴逆时针旋转 angle，z 不变
inline Vector3f rotate(const Vector3f& v, const float angle) {
    const auto [x, y] = rotate(v.x(), v.y(), angle);
    return {x, y, v.z()};
}

// 右手系 Z-Y-X 欧拉角：roll 绕 X、pitch 绕 Y、yaw 绕 Z，rad
struct EulerAngles {
    Angle yaw{};
    Angle pitch{};
    Angle roll{};
};

inline Quaternionf euler_to_quaternion(const EulerAngles& euler) {
    const Eigen::AngleAxisf yaw(euler.yaw, Vector3f::UnitZ());
    const Eigen::AngleAxisf pitch(euler.pitch, Vector3f::UnitY());
    const Eigen::AngleAxisf roll(euler.roll, Vector3f::UnitX());
    return Quaternionf(yaw * pitch * roll);
}

// 输入按模长归一化后换算
inline EulerAngles quaternion_to_euler(const Quaternionf& quaternion) {
    Quaternionf q = quaternion;
    const float norm = q.norm();
    if (std::isfinite(norm) && norm > 1.0e-6f)
        q.normalize();
    const float yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    const float pitch = std::asin(std::clamp(2 * (q.w() * q.y() - q.x() * q.z()), -1.0f, 1.0f));
    const float roll = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    return {.yaw = Angle(yaw), .pitch = Angle(pitch), .roll = Angle(roll)};
}
