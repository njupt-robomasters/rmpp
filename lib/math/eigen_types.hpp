#pragma once

#include "math/angle.hpp"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <algorithm>
#include <cmath>

using Vector2 = Eigen::Vector2f;
using Vector3 = Eigen::Vector3f;
using Vector4 = Eigen::Vector4f;
using Quaternion = Eigen::Quaternionf;
using Matrix3 = Eigen::Matrix3f;
using Matrix6 = Eigen::Matrix<float, 6, 6>;

// 右手系 Z-Y-X 欧拉角：roll 绕 X、pitch 绕 Y、yaw 绕 Z，rad
struct EulerAngles {
    Angle yaw{};
    Angle pitch{};
    Angle roll{};
};

inline Quaternion euler_to_quaternion(const EulerAngles& euler) {
    const Eigen::AngleAxisf yaw(euler.yaw, Vector3::UnitZ());
    const Eigen::AngleAxisf pitch(euler.pitch, Vector3::UnitY());
    const Eigen::AngleAxisf roll(euler.roll, Vector3::UnitX());
    return Quaternion(yaw * pitch * roll);
}

// 输入按模长归一化后换算
inline EulerAngles quaternion_to_euler(const Quaternion& quaternion) {
    Quaternion q = quaternion;
    const float norm = q.norm();
    if (std::isfinite(norm) && norm > 1.0e-6f)
        q.normalize();
    const float yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    const float pitch = std::asin(std::clamp(2 * (q.w() * q.y() - q.x() * q.z()), -1.0f, 1.0f));
    const float roll = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    return {.yaw = Angle(yaw), .pitch = Angle(pitch), .roll = Angle(roll)};
}
