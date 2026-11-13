#pragma once

#include "math/eigen.hpp"
#include "motor/motor.hpp"

// 电机功率模型：P = Σ(kt·I·ω + 0.75·R·I²)，R 为线间电阻（星形接法三相铜耗 1.5·(R/2)·I²），kt = max_torque / max_current
// 无状态，供各底盘共用；电流分平移、旋转两组衰减：I = gain.xy·I_xy + gain.z·I_z
// 国际单位制：功率 W，电流 A，电阻 Ω
class MotorPower {
  public:
    // 电流衰减系数，[0, 1]
    struct Gain {
        float xy = 1; // 平移
        float z = 1;  // 旋转
    };

    // 四个电机按反馈估计的当前功率；离线电机反馈已清零
    static float estimate(Motor* const motor[4], float resistance);

    // 求预测功率不超过 ref 的两组衰减系数并设置各电机电流；is_spin 时先衰减旋转，否则两组同值
    // 不能驱动的电机由调用方传入零电流
    static Gain limit_current(Motor* const motor[4], float resistance, const Vector4f& current_xy, const Vector4f& current_z, bool is_spin, float ref);

  private:
    // [0, 1] 内满足 a·g² + b·g + c ≤ 0 的最大 g，a ≥ 0；无解时为零
    static float solve_gain(float a, float b, float c);
};
