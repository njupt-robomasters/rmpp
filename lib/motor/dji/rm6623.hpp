#pragma once

#include "motor/motor.hpp"

class RM6623 : public Motor {
  public:
    // 型号默认物理参数
    static constexpr float MAX_CURRENT = 5;
    static constexpr float MAX_TORQUE = 1.9f;
    static constexpr float KT = 0.38f;
    static constexpr float REDUCTION = 1;

    explicit RM6623(const Config& config);
    bool write_can_frame(CanFrame& frame) const;
    void send_calibrate_cmd() const;

  private:
    void can_callback(const CanFrame& frame);

    Timer speed_timer_;
    float position_last_ = 0;
    bool has_position_ = false;
};
