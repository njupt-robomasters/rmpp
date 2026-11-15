#pragma once

// 项目唯一的 Eigen 入口，其他文件不直接包含 <Eigen/...>
// Eigen 模板在此按 O2 实例化：-Og 不内联，表达式模板逐层成为函数调用，EKF 慢约 10 倍；调用方仍按自身优化级别编译
#pragma GCC push_options
#pragma GCC optimize("O2")
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#pragma GCC pop_options

// 定长 float 类型，直接使用 Eigen 名称，不加前缀
using Eigen::Matrix3f;
using Eigen::Quaternionf;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Matrix6f = Eigen::Matrix<float, 6, 6>;
