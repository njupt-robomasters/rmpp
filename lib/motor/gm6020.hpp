#pragma once

#include "math/unit.hpp"
#include "motor/motor.hpp"

class GM6020 : public Motor {
  public:
    // 型号默认物理参数
    static constexpr float MAX_CURRENT = 3.0f;
    static constexpr float MAX_TORQUE = 2.223f;
    static constexpr float REDUCTION = 1.0f;
    static constexpr float MAX_VOLTAGE = 25.0f;        // 电压帧满量程，V
    static constexpr float KE = 1.0f / (13.33f * rpm); // 反电动势常数，V/(rad/s)，手册转速常数 13.33 rpm/V

    // ke 为电压帧反电动势前馈系数，V/(rad/s)；0 不补偿，需要时传 KE 或按比例缩小
    explicit GM6020(const Config& config, float ke = 0);

    // 电压帧 0x1FF/0x2FF 把 current_ref 按 3 A ↔ 25 V 换算为电压并叠加 ke·转速，电流帧 0x1FE/0x2FE 直接发电流
    void write_can_frame(CanFrame& frame) const;

  public:
    float motor_temperature = 0; // 电机温度，℃

  private:
    const float ke_;

    void can_callback(const CanFrame& frame);
};
