#pragma once

#include "motor/motor.hpp"

class M3508 : public Motor {
  public:
    // 型号默认物理参数
    static constexpr float MAX_CURRENT = 20.0f;
    static constexpr float MAX_TORQUE = 6.0f;
    static constexpr float KT = 0.3f;
    static constexpr float REDUCTION = 19.2032086f;

    float motor_temperature = 0; // 电机温度，℃

    explicit M3508(const Config& config);
    bool write_can_frame(CanFrame& frame) const;
    void send_quick_set_id() const;

  private:
    void can_callback(const CanFrame& frame);
};
