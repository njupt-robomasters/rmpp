#pragma once

#include "motor/motor.hpp"

class M2006 : public Motor {
  public:
    // 型号默认物理参数
    static constexpr float MAX_CURRENT = 10.0f;
    static constexpr float MAX_TORQUE = 1.8f;
    static constexpr float KT = 0.18f;
    static constexpr float REDUCTION = 36.0f;

    explicit M2006(const Config& config);
    bool write_can_frame(CanFrame& frame) const;

  private:
    void can_callback(const CanFrame& frame);
};
