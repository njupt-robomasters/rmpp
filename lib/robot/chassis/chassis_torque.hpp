#pragma once

#include "chassis.hpp"

// 力控制：底盘速度 PID 输出牵引力，按牵引力做功率控制，构型逆解为各轮力矩
class ChassisTorque : public Chassis {
  public:
    // 牵引力，云台系，fx 前正后负，fy 左正右负，fz 逆时针为正
    struct {
        float ref{}, measure{};
    } fx, fy, fz;

    // 功率衰减系数：平移、旋转
    float gain_xy = 1, gain_z = 1;

  protected:
    ChassisTorque(const Config& config, const Pid::Config* vxyz_pid_config, const std::array<Motor*, 4>& wheel);

    void clear_pid() override;

    void control() override;

    // 云台系牵引力目标换算到底盘系 (fx, fy, fz)
    std::tuple<float, float, float> force_ref() const;

    // 底盘系牵引力测量换算到云台系
    void set_force_measure(float fx, float fy, float fz);

    // 力学逆解：牵引力目标 → 各轮力矩
    virtual void backward() = 0;

  private:
    Pid vx_pid_, vy_pid_, vz_pid_; // 速度误差 m/s → 牵引力 N

    // 按功率限制和缓冲能量衰减牵引力，必须在逆解之前
    void power_control();
};
