#pragma once

#include "math/angle.hpp"
#include <Eigen/Geometry>
#include <Eigen/StdVector>

using Vector2 = Eigen::Vector2f;
using Vector3 = Eigen::Vector3f;
using Vector4 = Eigen::Vector4f;
using Quaternion = Eigen::Quaternionf;
using Matrix3 = Eigen::Matrix3f;
using Matrix6 = Eigen::Matrix<float, 6, 6>;

struct EulerAngles {
    Angle yaw{};
    Angle pitch{};
    Angle roll{};
};
