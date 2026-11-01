#pragma once

#include "chassis_speed.hpp"
#include "chassis_torque.hpp"

// 单个舵向：舵电机、舵向 PID 与目标，两种控制方式的舵轮底盘共用
class Steer {
  public:
    Motor& motor;

    // 舵向，0 朝前，逆时针为正
    Angle ref, measure;

    Steer(Motor& motor, const Pid::Config* pid_config);

    // 离线检测并读取舵向
    void update();

    // 按轮速度矢量设置目标舵向并闭环，返回投影到当前舵向的轮速，m/s；离线时返回零
    float control(float vx, float vy);

    // 清空 PID，下次控制从当前舵向开始
    void clear();

  private:
    static constexpr float MIN_V = 1 * cm; // 最小转舵速度，m/s

    Pid pid_; // 舵向误差 rad → 电流 A
    bool has_ref_ = false;
};

// 四舵轮底盘，速度控制；轮与舵顺序：左前、左后、右后、右前
class ChassisSteerSpeed : public ChassisSpeed {
  public:
    std::array<Steer, 4> steer;

    ChassisSteerSpeed(const Config& config, const Pid::Config* wheel_pid_config, const Pid::Config* steer_pid_config,
                      const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer);

    void update_ui(RobotUi& ui) override;

  private:
    void set_motors_enable(bool is_enable) override;

    void update_motors() override;

    bool can_drive(size_t i) const override;

    void clear_pid() override;

    void forward() override;

    std::array<float, 4> backward() override;
};

// 四舵轮底盘，力控制；轮与舵顺序：左前、左后、右后、右前
class ChassisSteerTorque : public ChassisTorque {
  public:
    std::array<Steer, 4> steer;

    ChassisSteerTorque(const Config& config, const Pid::Config* vxyz_pid_config, const Pid::Config* steer_pid_config,
                       const std::array<Motor*, 4>& wheel, const std::array<Motor*, 4>& steer);

    void update_ui(RobotUi& ui) override;

  private:
    void set_motors_enable(bool is_enable) override;

    void update_motors() override;

    bool can_drive(size_t i) const override;

    void clear_pid() override;

    void forward() override;

    void backward() override;
};
