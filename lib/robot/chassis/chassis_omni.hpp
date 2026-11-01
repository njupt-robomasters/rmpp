#pragma once

#include "chassis_torque.hpp"

// 全向轮底盘，力控制
class ChassisOmniTorque : public ChassisTorque {
  public:
    // wheel：左前、左后、右后、右前
    ChassisOmniTorque(const Config& config, const Pid::Config* vxyz_pid_config, const std::array<Motor*, 4>& wheel);

  private:
    void forward() override;

    void backward() override;
};
