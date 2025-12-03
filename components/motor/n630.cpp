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

void N630::SetEnable(const bool is_enable) {
    this->is_enable = is_enable;
}

void N630::SetBrakeMode(brake_mode_e brake_mode) {
    this->brake_mode = brake_mode;
}

void N630::SetSpeed(const UnitFloat<>& speed) {
    this->speed.ref = speed;
}

void N630::OnLoop() const {
    if (this->is_enable) {                   // 使能
        if (speed.ref == 0 * default_unit) { // 速度为0，判断刹车模式
            if (brake_mode == NO_BRAKE) {
                sendCurrentBrake(0);
            } else if (brake_mode == ALWAYS_BRAKE) {
                sendERPM(0);
            } else if (brake_mode == AUTO_BRAKE) {
                if (speed.measure == 0 * default_unit) {
                    sendCurrentBrake(0);
                } else {
                    sendERPM(0);
                }
            }
        } else {
            const float erpm = speed.ref.toFloat(rpm) * POLE_PAIR;
            sendERPM(erpm);
        }
    } else { // 失能
        sendCurrentBrake(0);
    }
}

void N630::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port != can_port) return;
    if ((id & 0xFF) != n630_id) return; // 必须要&FF，因为高位是命令码

    const uint8_t status = id >> 8;
    if (status == CAN_PACKET_STATUS) {
        if (dlc != 8) return;

        const auto erpm_i32 = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        const auto current_i16 = (int16_t)((data[4] << 8) | data[5]);
        const auto duty_i16 = (int16_t)((data[6] << 8) | data[7]);

        speed.measure = (float)erpm_i32 / POLE_PAIR * rpm;
        current = (float)current_i16 / 10.0f * A;
        duty = (float)duty_i16 / 1000.0f * pct;
    } else if (status == CAN_PACKET_STATUS_4) {
        if (dlc != 8) return;

        const auto temperate_mos_i16 = (int16_t)((data[0] << 8) | data[1]);
        const auto temperate_motor_i16 = (int16_t)((data[2] << 8) | data[3]);
        const auto current_in_i16 = (int16_t)((data[4] << 8) | data[5]);
        const auto pid_pos_i16 = (int16_t)((data[6] << 8) | data[7]);

        temperate_mos = (float)temperate_mos_i16 / 10.0f * C;
        temperate_motor = (float)temperate_motor_i16 / 10.0f * C;
        current_in = (float)current_in_i16 / 10.0f * A;
        pid_pos = (float)pid_pos_i16 / 50.0f * deg;
    } else if (status == CAN_PACKET_STATUS_5) {
        if (dlc != 6) return; // todo: dlc是6还是8

        const auto tachometer_i32 = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        const auto voltage_in_i16 = (int16_t)((data[4] << 8) | data[5]);

        tachometer = (float)tachometer_i32 * default_unit;
        voltage_in = (float)voltage_in_i16 / 10.0f * V;
    }
}

void N630::makeCANData(uint8_t data[4], const float value, const float scale) {
    const auto number = (uint32_t)(value * scale);
    data[0] = number >> 24;
    data[1] = number >> 16;
    data[2] = number >> 8;
    data[3] = number;
}

void N630::sendERPM(const float erpm) const {
    const uint32_t ext_id = n630_id | (CAN_PACKET_SET_RPM << 8);
    uint8_t data[8];
    makeCANData(data, erpm, 1);
    BSP::CAN::TransmitExt(can_port, ext_id, data, 4);
}

void N630::sendCurrentBrake(const float current) const {
    const uint32_t ext_id = n630_id | (CAN_PACKET_SET_CURRENT_BRAKE << 8);
    uint8_t data[8];
    makeCANData(data, current, 1000);
    BSP::CAN::TransmitExt(can_port, ext_id, data, 4);
}
