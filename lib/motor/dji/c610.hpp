#pragma once

#include "dji_motor.hpp"

class C610 : public DjiMotor {
  public:
    // 型号默认物理参数
    static constexpr Config::Drive DRIVE{.kt = 0.18f, .max_current = 10, .reduction = 36};

    // 构造与命令
    explicit C610(const Config& config);
    bool write_can_frame(CanFrame& frame) const override;

  private:
    void can_callback(const CanFrame& frame);
};
