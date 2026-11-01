#include "chassis_omni.hpp"
#include <cmath>

static const float SQRT2 = std::sqrt(2.0f);
static const float SQRT2DIV2 = SQRT2 / 2.0f;

ChassisOmniTorque::ChassisOmniTorque(const Config& config, const Pid::Config* vxyz_pid_config, const std::array<Motor*, 4>& wheel)
    : ChassisTorque(config, vxyz_pid_config, wheel) {}

void ChassisOmniTorque::forward() {
    // 速度正解
    const float v1 = wheel_speed(0), v2 = wheel_speed(1), v3 = wheel_speed(2), v4 = wheel_speed(3);
    set_speed_measure(SQRT2 * (-v1 - v2 + v3 + v4) / 4.0f,
                      SQRT2 * (+v1 - v2 - v3 + v4) / 4.0f,
                      (+v1 + v2 + v3 + v4) / 4.0f);

    // 力学正解
    const float f1 = wheel_force(0), f2 = wheel_force(1), f3 = wheel_force(2), f4 = wheel_force(3);
    set_force_measure(SQRT2DIV2 * (-f1 - f2 + f3 + f4),
                      SQRT2DIV2 * (+f1 - f2 - f3 + f4),
                      f1 + f2 + f3 + f4);
}

void ChassisOmniTorque::backward() {
    const auto [fx, fy, fz] = force_ref();
    wheel_[0]->set_torque((SQRT2 * (-fx + fy) + fz) / 4.0f * config_.wheel_radius);
    wheel_[1]->set_torque((SQRT2 * (-fx - fy) + fz) / 4.0f * config_.wheel_radius);
    wheel_[2]->set_torque((SQRT2 * (+fx - fy) + fz) / 4.0f * config_.wheel_radius);
    wheel_[3]->set_torque((SQRT2 * (+fx + fy) + fz) / 4.0f * config_.wheel_radius);
}
