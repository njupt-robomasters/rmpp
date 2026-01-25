#include "VESC.hpp"

VESC::VESC(const config_t& config, const vesc_config_t& vesc_config) : Motor(config), vesc_config(vesc_config) {
    // 注册CAN回调
    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);
}

void VESC::OnLoop() {
    if (dwt.send.PollTimeout(1 / config.can_send_freq)) { // CAN发送频率控制
        if (is_enable) { // 不检测电机在线，因为未必开了反馈报文
            if (speed.ref == 0 && unit::abs(speed.measure) < vesc_config.auto_release_rpm) {
                sendCurrentBrake(0); // 自动释放电机，避免持续的高频注入声音
            } else {
                const float erpm = speed.ref.toFloat(rpm) * (float)vesc_config.pole_pair;
                sendERPM(erpm);
            }
        } else {
            sendCurrentBrake(0); // 释放电机
        }
    }
}

void VESC::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // 端口、ID、长度校验
    if (port != config.can_port) return;
    if ((id & 0xFF) != config.master_id) return; // 必须要&FF，因为高位是命令码
    if (dlc != 8) return;

    const uint8_t status = id >> 8;
    if (status == CAN_PACKET_STATUS) {
        const auto erpm_i32 = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        const auto current_i16 = (int16_t)((data[4] << 8) | data[5]);
        const auto duty_i16 = (int16_t)((data[6] << 8) | data[7]);

        speed.measure = (float)erpm_i32 / (float)vesc_config.pole_pair * rpm;
        current.measure = (float)current_i16 / 10.0f * A;
        duty = (float)duty_i16 / 1000.0f * pct;
    } else if (status == CAN_PACKET_STATUS_4) {
        const auto temperate_mos_i16 = (int16_t)((data[0] << 8) | data[1]);
        const auto temperate_motor_i16 = (int16_t)((data[2] << 8) | data[3]);
        const auto current_in_i16 = (int16_t)((data[4] << 8) | data[5]);
        const auto pid_pos_i16 = (int16_t)((data[6] << 8) | data[7]);

        temperature.mos = (float)temperate_mos_i16 / 10.0f * C;
        temperature.motor = (float)temperate_motor_i16 / 10.0f * C;
        bus.current = (float)current_in_i16 / 10.0f * A;
        angle.measure = (float)pid_pos_i16 / 50.0f * deg;
    } else if (status == CAN_PACKET_STATUS_5) {
        // const auto tachometer_i32 = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        const auto voltage_in_i16 = (int16_t)((data[4] << 8) | data[5]);

        bus.voltage = (float)voltage_in_i16 / 10.0f * V;
    }
}

void VESC::makeCANData(uint8_t data[4], const float value, const float scale) {
    const auto number = (uint32_t)(value * scale);
    data[0] = number >> 24;
    data[1] = number >> 16;
    data[2] = number >> 8;
    data[3] = number;
}

void VESC::sendERPM(const float erpm) const {
    const uint32_t ext_id = config.slave_id | (CAN_PACKET_SET_RPM << 8);
    uint8_t data[8];
    makeCANData(data, erpm, 1);
    BSP::CAN::TransmitExt(config.can_port, ext_id, data, 4);
}

void VESC::sendCurrentBrake(const float current) const {
    const uint32_t ext_id = config.slave_id | (CAN_PACKET_SET_CURRENT_BRAKE << 8);
    uint8_t data[8];
    makeCANData(data, current, 1000);
    BSP::CAN::TransmitExt(config.can_port, ext_id, data, 4);
}