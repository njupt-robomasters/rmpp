#pragma once

#include "motor/motor.hpp"

class RM6623 : public Motor {
  public:
    // 型号默认物理参数
    static constexpr float MAX_CURRENT = 5;
    static constexpr float MAX_TORQUE = 1.9f;
    static constexpr float REDUCTION = 1;

  private:
    // 反馈不含转速，由相邻两帧位置差分
    float position_last = 0;
    bool has_position = false;
    Timer speed_timer;

  public:
    explicit RM6623(const Config& config);

    void write_can_frame(CanFrame& frame) const;

    void send_calibrate_cmd() const;

  private:
    void can_callback(const CanFrame& frame);
};
