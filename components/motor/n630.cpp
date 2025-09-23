#include "n630.hpp"
#include "bsp.hpp"

N630::N630(const uint8_t can_port, const uint32_t n630_id) :
    can_port(can_port), n630_id(n630_id) {}

void N630::makeCANData(uint8_t buffer[4], const float value, const float scale) {
    const uint32_t number = value * scale;
    buffer[0] = number >> 24;
    buffer[1] = number >> 16;
    buffer[2] = number >> 8;
    buffer[3] = number;
}

void N630::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
}

void N630::SetSpeed(const UnitFloat<rpm> speed) {
    this->speed = speed;
}

void N630::Update() {
    if (this->is_enable) {
        uint8_t buffer[8];
        makeCANData(buffer, speed.get(rpm), 1);
        uint32_t ext_id = n630_id | (CAN_PACKET_SET_RPM << 8);
        BSP::CAN::TransmitExt(can_port, ext_id, buffer, 4);
    } else {
        uint8_t buffer[8];
        makeCANData(buffer, 0, 1000);
        uint32_t ext_id = n630_id | (CAN_PACKET_SET_CURRENT_BRAKE << 8);
        BSP::CAN::TransmitExt(can_port, ext_id, buffer, 4);
    }
}
