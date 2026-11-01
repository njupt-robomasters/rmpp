#include "chassis_speed.hpp"
#include <algorithm>
#include <cmath>

ChassisSpeed::ChassisSpeed(const Config& config, const Pid::Config* wheel_pid_config, const std::array<Motor*, 4>& wheel)
    : Chassis(config, wheel), wheel_pid_{Pid(wheel_pid_config), Pid(wheel_pid_config), Pid(wheel_pid_config), Pid(wheel_pid_config)} {}

void ChassisSpeed::clear_pid() {
    Chassis::clear_pid();
    for (Pid& pid : wheel_pid_)
        pid.clear();
}

void ChassisSpeed::control() {
    const std::array<float, 4> v = backward();

    // 轮速 PID，不能驱动的轮清空 PID 并输出零
    for (size_t i = 0; i < wheel_.size(); i++) {
        Motor& wheel = *wheel_[i];
        if (!can_drive(i)) {
            wheel_pid_[i].clear();
            wheel.set_current(0);
            continue;
        }
        wheel.set_current(wheel_pid_[i].calculate(v[i] / config_.wheel_radius - wheel.speed));
    }

    power_control();
}

void ChassisSpeed::power_control() {
    gain = 1;

    // 裁判系统不限功率时读出 65535，此时缓冲能量也异常，必须跳过
    if (power.limit == 65535)
        return;

    // 电流整体乘 gain 后预测功率为 a·gain² + b·gain，解 a·gain² + b·gain - limit = 0
    float a = 0, b = 0;
    for (const Motor* wheel : wheel_) {
        a += copper_loss(wheel->current_ref);
        b += wheel->torque_ref * wheel->speed;
    }
    const float c = -power.limit;
    if (a + b + c > 0) {
        if (a > 0)
            gain = (-b + std::sqrtf(b * b - 4 * a * c)) / (2 * a);
        else if (b > 0) // 零电流时避免除零
            gain = -c / b;
    }
    gain = std::clamp(gain, 0.0f, 1.0f);

    // 缓冲能量低于 30 J 停止驱动，60 J 以上不衰减
    gain *= std::clamp((power.buffer_energy - 30) / 30, 0.0f, 1.0f);

    for (Motor* wheel : wheel_)
        wheel->set_current(wheel->current_ref * gain);
}
