#pragma once

#include "chassis.hpp"

// 速度控制：构型逆解为各轮线速度，轮速 PID 输出电流，按电流二次式整体衰减功率
class ChassisSpeed : public Chassis {
  public:
    float gain = 1; // 功率衰减系数

  protected:
    ChassisSpeed(const Config& config, const Pid::Config* wheel_pid_config, const std::array<Motor*, 4>& wheel);

    void clear_pid() override;

    void control() override;

    // 速度逆解：底盘目标 → 各轮线速度目标，m/s
    virtual std::array<float, 4> backward() = 0;

  private:
    std::array<Pid, 4> wheel_pid_; // 轮转速误差 rad/s → 电流 A

    // 按功率限制和缓冲能量衰减轮电流，必须在轮速 PID 之后
    void power_control();
};
