#pragma once

#include "dji_motor.hpp"

class GM6020 : public DjiMotor {
  public:
    // 型号默认物理参数
    static constexpr Config::Drive DRIVE{.kt = 0.741f, .max_current = 3, .reduction = 1};

    // 公开反馈
    float temperature = 0; // 电机温度，℃

    // 构造与命令
    explicit GM6020(const Config& config);
    bool write_can_frame(CanFrame& frame) const override;

  private:
    void can_callback(const CanFrame& frame);
};
