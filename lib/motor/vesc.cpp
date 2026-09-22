#include "vesc.hpp"
#include "math/unit.hpp"
#include <cmath>

Vesc::Vesc(const Config& config, const VescConfig& vesc_config) : Motor(config), vesc_config(vesc_config) {
    // 设置电机默认参数
    if (this->config.reduction == 0)
        this->config.reduction = 1.0f;
    if (this->config.kt == 0)
        this->config.kt = 1.0f;

    // 注册CAN回调
    config.can.add_rx_callback([this](const CanFrame& frame) {
        this->callback(frame.id, frame.data.data(), frame.size);
    });
}

void Vesc::on_loop() {
    // 电机断联检测
    if (timer_connect_.get_dt() > config.timeout) {
        is_connect = false;
    }
}

void Vesc::send_can_cmd() {
    if (is_enable) { // 不检测电机在线，因为未必开了反馈报文
        if (speed.ref == 0 && std::fabsf(speed.measure) < vesc_config.auto_release_speed) {
            send_current_brake(0); // 自动释放电机，避免持续的高频注入声音
        } else {
            const float erpm = speed.ref / (rpm) * (float)vesc_config.pole_pair;
            send_erpm(erpm);
        }
    } else {
        send_current_brake(0); // 释放电机
    }
}

void Vesc::callback(const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // ID、长度校验
    if ((id & 0xFF) != config.master_id)
        return; // 必须要&FF，因为高位是命令码
    if (dlc != 8)
        return;

    // 更新电机断联检测
    is_connect = true;
    timer_connect_.update_dt();
    ;

    const uint8_t status = id >> 8;
    if (status == CAN_PACKET_STATUS) {
        const auto erpm_i32 = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        const auto current_i16 = (int16_t)((data[4] << 8) | data[5]);
        const auto duty_i16 = (int16_t)((data[6] << 8) | data[7]);

        speed.measure = (float)erpm_i32 / (float)vesc_config.pole_pair * rpm;
        current.measure = (float)current_i16 / 10.0f;
        duty = (float)duty_i16 / 1000.0f * pct;
    } else if (status == CAN_PACKET_STATUS_4) {
        const auto temperate_mos_i16 = (int16_t)((data[0] << 8) | data[1]);
        const auto temperate_motor_i16 = (int16_t)((data[2] << 8) | data[3]);
        const auto current_in_i16 = (int16_t)((data[4] << 8) | data[5]);
        const auto pid_pos_i16 = (int16_t)((data[6] << 8) | data[7]);

        temperature.mos = (float)temperate_mos_i16 / 10.0f;
        temperature.motor = (float)temperate_motor_i16 / 10.0f;
        bus.current = (float)current_in_i16 / 10.0f;
        angle.measure = (float)pid_pos_i16 / 50.0f * deg;
    } else if (status == CAN_PACKET_STATUS_5) {
        // const auto tachometer_i32 = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        const auto voltage_in_i16 = (int16_t)((data[4] << 8) | data[5]);

        bus.voltage = (float)voltage_in_i16 / 10.0f;
    }
}

void Vesc::make_can_data(uint8_t data[4], const float value, const float scale) {
    const auto number = (int32_t)(value * scale);
    data[0] = number >> 24;
    data[1] = number >> 16;
    data[2] = number >> 8;
    data[3] = number;
}

void Vesc::send_erpm(const float erpm) const {
    const uint32_t ext_id = config.slave_id | (CAN_PACKET_SET_RPM << 8);
    std::array<uint8_t, 8> data{};
    make_can_data(data.data(), erpm, 1);
    config.can.send({.id = ext_id, .data = data, .size = 4, .extended = true});
}

void Vesc::send_current_brake(const float current) const {
    const uint32_t ext_id = config.slave_id | (CAN_PACKET_SET_CURRENT_BRAKE << 8);
    std::array<uint8_t, 8> data{};
    make_can_data(data.data(), current, 1000);
    config.can.send({.id = ext_id, .data = data, .size = 4, .extended = true});
}