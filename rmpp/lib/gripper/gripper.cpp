#include "gripper.hpp"

Gripper::Gripper(const config_t& config) : config(config) {
    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);
}

void Gripper::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
}

void Gripper::SendCanCmd() {
    // 构造CAN数据帧
    // data[0]: 状态（0=松开，1=夹紧）
    // data[1]: 最大速度（度/秒）
    // data[2]: 电流高位（毫安）
    // data[3]: 电流低位（毫安）
    // data[4-7]: 保留（0x00）
    uint8_t data[8]{};
    data[0] = is_close;
    data[1] = (int8_t)speed.toFloat(deg_s);
    data[2] = (int16_t)current.ref.toFloat(mA) >> 8;
    data[3] = (int8_t)current.ref.toFloat(mA);

    BSP::CAN::Transmit(config.can_port, config.cmd_id, data, 4);
}

void Gripper::Open(const UnitFloat<>& speed) {
    this->is_close = false;
    this->speed = speed;
    this->current.ref = 0 * A; // 松开时不需要电流控制
}

void Gripper::Close(const UnitFloat<>& speed, const UnitFloat<>& current) {
    this->is_close = true;
    this->speed = speed;
    this->current.ref = current;
}

void Gripper::OnLoop() {
    if (dwt_connect.GetDT() > config.timeout) {
        is_connect = false;
    }
}

void Gripper::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != config.can_port) return;
    if (id != config.feedback_id) return;
    if (data[0] != 0x00 && data[0] != 0x01) return;

    is_connect = true;
    dwt_connect.UpdateDT();

    current.measure1 = (int16_t)((data[1] << 8) | data[2]) * mA;
    current.measure2 = (int16_t)((data[3] << 8) | data[4]) * mA;
}