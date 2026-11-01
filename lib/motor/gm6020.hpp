#pragma once

#include "math/unit.hpp"
#include "motor/motor.hpp"

class GM6020 : public Motor {
  public:
    // 型号默认物理参数
    static constexpr float MAX_CURRENT = 3.0f;
    static constexpr float MAX_TORQUE = 2.223f;
    static constexpr float KT = 0.741f;
    static constexpr float REDUCTION = 1.0f;
    static constexpr float MAX_VOLTAGE = 25.0f;        // 电压帧满量程，V
    static constexpr float KE = 1.0f / (13.33f * rpm); // 反电动势常数，V/(rad/s)，手册转速常数 13.33 rpm/V

    // 电压帧反电动势前馈：电压 += gain·ke·转速；gain 为零时不补偿
    struct BackEmf {
        float gain = 0;
        float ke = KE;
    };

    float motor_temperature = 0; // 电机温度，℃

    // 嵌套类默认成员初始化在类内不可用作默认实参，不补偿时用单参数构造
    explicit GM6020(const Config& config);
    GM6020(const Config& config, const BackEmf& back_emf);

    // 电压帧 0x1FF/0x2FF 把 current_ref 按 3 A ↔ 25 V 换算为电压并叠加前馈，电流帧 0x1FE/0x2FE 直接发电流
    bool write_can_frame(CanFrame& frame) const;

  private:
    const BackEmf back_emf_;

    void can_callback(const CanFrame& frame);
};
