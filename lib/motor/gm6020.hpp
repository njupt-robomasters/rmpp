#pragma once

#include "motor/motor.hpp"

class GM6020 : public Motor {
  public:
    // 型号默认物理参数
    static constexpr float MAX_CURRENT = 3.0f;
    static constexpr float MAX_TORQUE = 2.223f;
    static constexpr float KT = 0.741f;
    static constexpr float REDUCTION = 1.0f;

    float motor_temperature = 0; // 电机温度，℃

    explicit GM6020(const Config& config);
    bool write_can_frame(CanFrame& frame) const;

  private:
    void can_callback(const CanFrame& frame);
};
