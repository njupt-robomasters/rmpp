#pragma once

#include "motor.hpp"

class GM6020 : public Motor {
public:
    static constexpr UnitFloat MAX_CU0RRENT = 3.0f * A;
    static constexpr uint16_t MAX_VOLTAGE_CMD = 25000;
    static constexpr float REDUCTION = 36.0f;
    static constexpr UnitFloat Kt = 0.3f * Nm_A;

    // CAN通信参数
    const uint16_t can_port;
    const uint8_t motor_id;         // 电机ID，根据手册设置，范围：1~7
    const uint16_t control_can_id;  // 控制报文ID：motor_id 1~4 => 0x1FF；motor_id 5~7 => 0x2FF
    const uint16_t feedback_can_id; // 电机反馈报文ID：0x204 + motor_id

    GM6020(uint8_t can_port, uint8_t motor_id);

    int16_t GetVoltageCmd() const;

private:
    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};
