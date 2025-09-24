#include "n630.hpp"
#include "bsp.hpp"

N630::N630(const uint8_t can_port, const uint32_t n630_id) :
    can_port(can_port), n630_id(n630_id) {
    BSP::CAN::RegisterCallback(std::bind(&N630::callback, this,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3,
                                         std::placeholders::_4));
}

void N630::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != can_port) return;
    if (id & 0xFF != n630_id) return;

    const uint8_t status = id >> 8;
    if (status == CAN_PACKET_STATUS) {
        const int32_t erpm = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
        speed.measure = (float)erpm / (POLES / 2) * rpm;
        current = (float)(int16_t)(data[4] << 8 | data[5]) / 10.0f;
        duty = (float)(int16_t)(data[6] << 8 | data[7]) / 1000.0f;
    } else if (status == CAN_PACKET_STATUS_4) {
        temperate_mos = (float)(int16_t)(data[0] << 8 | data[1]) / 10.0f;
        temperate_motor = (float)(int16_t)(data[2] << 8 | data[3]) / 10.0f;
        current_in = (float)(int16_t)(data[4] << 8 | data[5]) / 10.0f;
    }
}

void N630::makeCANData(uint8_t buffer[4], const float value, const float scale) {
    const uint32_t number = (uint32_t)(value * scale);
    buffer[0] = number >> 24;
    buffer[1] = number >> 16;
    buffer[2] = number >> 8;
    buffer[3] = number;
}

void N630::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
}

void N630::SetSpeed(const UnitFloat<>& speed) {
    this->speed.ref = speed;
}

void N630::Update() {
    if (this->is_enable && speed.ref != 0) {
        uint8_t buffer[8];
        const float erpm = speed.ref.get(rpm) * POLES / 2;;
        makeCANData(buffer, erpm, 1);
        const uint32_t ext_id = n630_id | (CAN_PACKET_SET_RPM << 8);
        BSP::CAN::TransmitExt(can_port, ext_id, buffer, 4);
    } else {
        uint8_t buffer[8];
        makeCANData(buffer, 0, 1000);
        const uint32_t ext_id = n630_id | (CAN_PACKET_SET_CURRENT_BRAKE << 8);
        BSP::CAN::TransmitExt(can_port, ext_id, buffer, 4);
    }
}
