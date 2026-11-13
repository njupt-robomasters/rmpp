#include "motor_power.hpp"
#include <algorithm>
#include <cmath>

float MotorPower::estimate(Motor* const motor[4], const float resistance) {
    const float k = 0.75f * resistance;
    float power = 0;
    for (int i = 0; i < 4; i++)
        power += motor[i]->torque * motor[i]->speed + k * motor[i]->current * motor[i]->current;
    return power;
}

MotorPower::Gain MotorPower::limit_current(Motor* const motor[4], const float resistance, const Vector4f& current_xy, const Vector4f& current_z, const bool is_spin, const float ref) {
    // 预测功率按两组系数展开：二次项、交叉项与一次项系数
    const float k = 0.75f * resistance;
    float xx = 0, zz = 0, xz = 0, x = 0, z = 0;
    for (int i = 0; i < 4; i++) {
        const auto& drive = motor[i]->config.drive;
        const float kt_speed = drive.max_torque / drive.max_current * motor[i]->speed;
        xx += k * current_xy[i] * current_xy[i];
        zz += k * current_z[i] * current_z[i];
        xz += 2 * k * current_xy[i] * current_z[i];
        x += kt_speed * current_xy[i];
        z += kt_speed * current_z[i];
    }

    // 平移不衰减解旋转；停止旋转仍超功率时再解平移
    Gain gain;
    if (is_spin) {
        gain.z = solve_gain(zz, xz + z, xx + x - ref);
        if (gain.z == 0)
            gain.xy = solve_gain(xx, x, -ref);
    } else {
        gain.xy = gain.z = solve_gain(xx + zz + xz, x + z, -ref);
    }

    for (int i = 0; i < 4; i++)
        motor[i]->set_current(gain.xy * current_xy[i] + gain.z * current_z[i]);
    return gain;
}

float MotorPower::solve_gain(const float a, const float b, const float c) {
    if (a + b + c <= 0)
        return 1;
    if (a > 0) {
        const float delta = b * b - 4 * a * c;
        if (delta < 0)
            return 0;
        return std::clamp((-b + std::sqrt(delta)) / (2 * a), 0.0f, 1.0f);
    }
    if (b > 0) // 零电流时为一次式
        return std::clamp(-c / b, 0.0f, 1.0f);
    return 0;
}
