#include "gripper.hpp"
#include "bsp/bsp_can.hpp"

Gripper::Gripper(const config_t& config) : config(config) {
    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);
}

void Gripper::SetEnable(bool is_enable) {
    this->is_enable = is_enable;
}

void Gripper::Open(uint8_t speed) {
    this->speed = speed;
    this->is_open = true;
    this->current = 0;  // 松开时不需要电流控制
    
    // 构造CAN数据帧
    // data[0]: 0x00 表示松开
    // data[1]: 速度（度/秒）
    // data[2-3]: 电流为0（松开时不控制电流）
    // data[4-7]: 保留字节
    uint8_t data[8] = {
        0x00,
        speed,
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    };

    BSP::CAN::Transmit(config.can_port, config.cmd_id, data, 8);
}

void Gripper::Close(uint8_t speed, uint16_t current) {
    this->speed = speed;
    this->is_open = false;
    this->current = current;
    
    // 构造CAN数据帧
    // data[0]: 0x01 表示夹紧
    // data[1]: 速度（度/秒）
    // data[2-3]: 电流值（高位在前）
    // data[4-7]: 保留字节
    uint8_t data[8] = {
        0x01,
        speed,
        static_cast<uint8_t>(current >> 8),
        static_cast<uint8_t>(current & 0xFF),
        0x00, 0x00, 0x00, 0x00
    };

    BSP::CAN::Transmit(config.can_port, config.cmd_id, data, 8);
}

void Gripper::OnLoop() {
    if (dwt_connect.GetDT() > config.timeout) {
        is_connect = false;
    }
}

void Gripper::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != config.can_port) return;
    if (id != config.feedback_id) return;

    is_connect = true;
    dwt_connect.UpdateDT();

    feedback.is_open = (data[0] == 0x00);
    feedback.servo1_current = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    feedback.servo2_current = (static_cast<uint16_t>(data[3]) << 8) | data[4];
}
