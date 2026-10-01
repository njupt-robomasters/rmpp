#pragma once

#include "dji_motor.hpp"

class C620 : public DjiMotor {
  public:
    // 型号默认物理参数
    static constexpr Config::Drive DRIVE{.kt = 0.3f, .max_current = 20, .reduction = 3591.0f / 187.0f};

    // 公开反馈
    float temperature = 0; // 电机温度，℃

    // 构造与命令
    explicit C620(const Config& config);
    bool write_can_frame(CanFrame& frame) const override;

    void send_quick_set_id() const;

  private:
    void can_callback(const CanFrame& frame);
};
