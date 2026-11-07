#pragma once

#include "math/eigen_types.hpp"
#include "motor/motor.hpp"
#include "super_cap.hpp"

// 功率限制：底盘轮电机功率估计、缓冲能量功率预算、超电、两组电流衰减；由矩阵底盘与舵轮底盘按值持有
// 各轮电流分平移、旋转两组衰减：I = gain_xy·I_xy + gain_z·I_z
// 预测功率 Σ(铜耗 + kt·I·ω) + error，不含舵电机
// 国际单位制：功率 W，能量 J，电流 A
class PowerLimiter {
  public:
    struct Config {
        float wheel_resistance = 0;   // 轮电机线间电阻，Ω；星形接法三相铜耗 1.5·(R/2)·iq² = 0.75·R·iq²
        float error_fc = 5;           // 功率偏差（实测 − 估计）低通截止频率，Hz
        float buffer_gain_max = 1.1f; // 缓冲能量满 60 J 时功率预算相对裁判上限的倍数；30 J 为零，其间线性
    };

    float estimate = 0; // 估计功率：轮电机机械功率与铜耗
    float measure = 0;  // 超电实测底盘负载功率，超电未生效时为零
    float error = 0;    // 低通后的实测 − 估计，叠加到功率预测；超电未生效时为零
    float budget = 0;   // 按缓冲能量折算的功率预算，也是超电电池侧目标
    float max = 0;      // 本周期生效的功率上限：预算，BOOST 叠加电容可补偿功率
    float gain_xy = 1;  // 平移电流衰减系数
    float gain_z = 1;   // 旋转电流衰减系数

    // super_cap 由本类更新并设置电池侧目标功率；为空时按估计功率控制
    PowerLimiter(SuperCap* super_cap, const Config& config);

    // 设置裁判功率上限，65535 表示不限功率
    void set_limit(float limit);

    // 设置裁判缓冲能量，上限 60 J，10 Hz 更新
    void set_buffer_energy(float buffer_energy);

    // 每个控制周期调用：功率估计、功率预算、超电更新与本周期功率上限，失能时也调用以便停车充电
    void update(Motor* const wheel[4]);

    // 求两组衰减系数并设置各轮电流 A；is_spin 时先衰减旋转，否则两组同值
    // 不能驱动的轮由调用方传入零电流
    void control(Motor* const wheel[4], const Vector4f& current_xy, const Vector4f& current_z, bool is_spin);

  private:
    static constexpr float BUFFER_CUTOFF = 30; // 缓冲能量低于此值切断输出，J
    static constexpr float BUFFER_FULL = 60;   // 裁判缓冲能量上限，J

    SuperCap* const super_cap_; // 超级电容，可为空
    const Config config_;
    float limit_ = 120;        // 裁判功率上限，65535 表示不限
    float buffer_energy_ = 60; // 缓冲能量
    Timer timer_;              // 功率偏差低通的周期

    // [0, 1] 内满足 a·g² + b·g + c ≤ 0 的最大 g，a ≥ 0；无解时为零
    static float solve_gain(float a, float b, float c);
};
