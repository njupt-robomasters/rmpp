#pragma once

#include "motor.hpp"

class MDJI : public Motor {
public:
    // CAN通信参数
    const uint8_t can_port;         // CAN接口
    const uint32_t feedback_can_id; // CAN反馈报文ID
    const UnitFloat<A> current_max; // 最大电流
    const uint16_t current_cmd_max; // CAN通信最大电流对应的值

    // 电机温度
    UnitFloat<C> temperature_motor;

    MDJI(uint8_t can_port, uint32_t feedback_can_id, const UnitFloat<>& current_max, uint16_t current_cmd_max);

    // 获取CAN报文电流值（todo: 包装一拖四控制）
    int16_t GetCurrentCMD() const;

private:
    // CAN回调函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);
};
