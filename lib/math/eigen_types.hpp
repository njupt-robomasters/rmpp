#pragma once

#include <Eigen/Geometry>

// 定长 float 类型，直接使用 Eigen 名称，不加前缀
using Eigen::Matrix3f;
using Eigen::Quaternionf;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Matrix6f = Eigen::Matrix<float, 6, 6>;
