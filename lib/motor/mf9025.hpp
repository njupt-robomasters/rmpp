#pragma once

#include "motor.hpp"

class MF9025 : public Motor {
public:
    static constexpr UnitFloat Kt = 0.32f * Nm_A;

    UnitFloat<C> temperature_motor; // 电机温度

    MF9025(uint8_t can_port, uint8_t motor_id);

    // 需要在循环中调用
    void OnLoop();

private:
    // CAN通信参数
    const uint8_t can_port;
    const uint8_t motor_id; // 控制报文和反馈报文均为0x140+motor_id

    uint32_t send_cnt = 0; // 用于定期发送使能命令

    // CAN回调函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    // 发送电机使能
    void sendEnable() const;

    // 发送电机失能
    void sendDisable() const;

    // 触发状态读取
    void sendReadState() const;

    // 发送电流
    void sendCurrent() const;
};