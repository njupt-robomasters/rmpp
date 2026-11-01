#pragma once

#include "chassis_torque.hpp"

// 麦克纳姆轮底盘，力控制
class ChassisMecanumTorque : public ChassisTorque {
  public:
    // wheel：左前、左后、右后、右前
    ChassisMecanumTorque(const Config& config, const Pid::Config* vxyz_pid_config, const std::array<Motor*, 4>& wheel);

  private:
    void forward() override;

    void backward() override;
};
