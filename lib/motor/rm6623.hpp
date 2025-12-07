#pragma once
#include "motor.hpp"

class RM6623 : public Motor {
public:
    // CAN通信参数
    const uint8_t can_port;
    const uint8_t motor_id;         // 电机ID，按照手册设置，范围0~7
    const uint16_t control_can_id;  // 电机控制报文ID（motor_id 0~3  → 0x1FF；motor_id 4~7 → 0x2FF）
    const uint16_t feedback_can_id; // 电机反馈报文ID（0x205 + motor_id）

    RM6623(uint8_t can_port, uint8_t motor_id);

    int16_t GetCurrentCmd() const;

    // 发送电机校准指令（更换电机/电调时使用）
    void SendCalibrateCmd() const;

private:
    // CAN接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};
