#pragma once

#include "dji_motor.hpp"

class RM6623 : public DjiMotor {
  public:
    // 型号默认物理参数
    static constexpr Config::Drive DRIVE{.kt = 0.38f, .max_current = 5, .reduction = 1};

    // 构造与命令
    explicit RM6623(const Config& config);
    bool write_can_frame(CanFrame& frame) const override;

    void send_calibrate_cmd() const;

  private:
    void can_callback(const CanFrame& frame);
};
