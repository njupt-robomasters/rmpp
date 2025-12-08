#pragma once

#include "motor.hpp"

class M2006 : public Motor {
public:
    static constexpr UnitFloat MAX_CURRENT = 10.0f * A;
    static constexpr uint16_t MAX_CURRENT_CMD = 10000;
    static constexpr float REDUCTION = 36.0f;
    static constexpr UnitFloat Kt = 0.3f * Nm_A;
    static constexpr float R = 0.461f / 2; // 相电阻（两相电阻/2）

    // CAN通信参数
    const uint16_t can_port;
    const uint8_t motor_id;         // 电机ID，根据手册设置，范围：1~8
    const uint16_t control_can_id;  // 控制报文ID：motor_id 1~4 => 0x200；motor_id 5~8 => 0x1FF
    const uint16_t feedback_can_id; // 电机反馈报文ID：0x200 + motor_id

    M2006(uint8_t can_port, uint8_t motor_id);

    int16_t GetCurrentCmd() const;

private:
    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};