#pragma once

#include "bsp/timer.hpp"
#include "math/eigen_types.hpp"
#include "motor/motor.hpp"
#include "power/super_cap.hpp"
#include <array>

// 底盘功率：轮电机功率估计、超电、功率与缓冲能量限制；矩阵底盘与舵轮底盘共用
// 各轮电流分平移、旋转两组衰减：I = gain_xy·I_xy + gain_z·I_z
// 预测功率 Σ(铜耗 + kt·I·ω) + error，不含舵电机
// 国际单位制：功率 W，能量 J，电流 A
class ChassisPower {
  public:
    struct Config {
        float wheel_resistance{};      // 轮电机线间电阻，Ω
        float error_fc = 5;            // 功率偏差（实测 − 估计）低通截止频率，Hz
        SuperCap* super_cap = nullptr; // 超级电容，由底盘更新并设置电池侧目标功率；为空时按估计功率控制
    };

    float limit = 120;        // 裁判功率上限，65535 表示不限
    float buffer_energy = 60; // 缓冲能量
    float estimate{};         // 估计功率：轮电机机械功率与铜耗
    float measure{};          // 超电实测底盘负载功率，超电未生效时为零
    float error{};            // 低通后的实测 − 估计，叠加到功率预测；超电未生效时为零
    float max{};              // 本周期生效的功率上限：超电按模式调整，否则等于 limit
    float gain_xy = 1;        // 平移电流衰减系数
    float gain_z = 1;         // 旋转电流衰减系数

    explicit ChassisPower(const Config& config);

    const Config& config() const;

    // 设置裁判功率上限，65535 表示不限功率
    void set_limit(float limit);

    // 设置缓冲能量
    void set_buffer_energy(float buffer_energy);

    // 每个控制周期调用：功率估计、超电更新与本周期功率上限，失能时也调用以便停车充电
    void update(const std::array<Motor*, 4>& wheel);

    // 求两组衰减系数并设置各轮电流 A；is_spin 时先衰减旋转，否则两组同值
    // 不能驱动的轮由调用方传入零电流
    void control(const std::array<Motor*, 4>& wheel, const Vector4& current_xy, const Vector4& current_z, bool is_spin);

  private:
    const Config config_;
    Timer timer_; // 功率偏差低通的周期

    // 单个轮电机铜耗系数，铜耗 = k·iq²
    float copper_loss_k() const;
};
