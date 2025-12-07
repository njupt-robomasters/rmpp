#pragma once

#include "motor.hpp"

class M3508 : public Motor {
public:
    static constexpr UnitFloat MAX_CURRENT = 20.0f * A;
    static constexpr uint16_t MAX_CURRENT_CMD = 16384;
    static constexpr float REDUCTION = 3591.0f / 187.0f;
    static constexpr UnitFloat Kt = 0.18f * Nm_A;

    // CAN通信参数
    const uint16_t can_port;
    const uint8_t motor_id;         // 电机ID，根据手册设置，范围：1~8
    const uint16_t control_can_id;  // 控制报文ID：motor_id 1~4 => 0x200；motor_id 5~8 => 0x1FF
    const uint16_t feedback_can_id; // 反馈报文ID：0x200 + motor_id

    // 电机温度
    UnitFloat<C> temperature_motor;

    // 电流衰减系数，用于功率控制
    UnitFloat<pct> current_ratio = 100.0f * pct;

    M3508(uint8_t can_port, uint8_t motor_id);

    // 设置电流衰减，用于功率控制（todo: 解耦功率控制）
    void SetCurrentRatio(const UnitFloat<>& current_ratio);

    int16_t GetCurrentCmd() const;

private:
    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};
