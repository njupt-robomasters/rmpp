#pragma once

#include "math/unit.hpp"
#include "motor/motor.hpp"

// N630/P630 电调，CAN 协议与 M3508（C620）一致；物理参数暂按 M3508 取值
class X630 : public Motor {
  public:
    // 型号默认物理参数，同 M3508
    static constexpr float MAX_CURRENT = 20.0f;
    static constexpr float MAX_TORQUE = 6.0f;
    static constexpr float REDUCTION = 19.2032086f;

    // HFI 静音：静止附近 HFI 有高频噪音，目标电流与转速均低于进入阈值时静音（输出零电流），目标电流高于退出阈值才恢复
    struct Mute {
        float enter_current = 1.0f; // A
        float exit_current = 2.0f;  // A
        float enter_speed = 10 * rpm;
    };

  private:
    float motor_temperature = 0; // 电机温度，℃
    const float enter_current, exit_current, enter_speed;
    bool is_mute = false;

  public:
    X630(const Config& config, const Mute& mute);

    // 设置后按静音滞回判断，静音时 current_ref、torque_ref 置零
    void set_current(float current) override;

    void set_torque(float torque) override;

    void write_can_frame(CanFrame& frame) const;

  private:
    void can_callback(const CanFrame& frame);

    void apply_mute();
};
