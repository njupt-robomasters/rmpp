#include "vesc.hpp"
#include "math/unit.hpp"
#include <cmath>

Vesc::Vesc(const Config& config)
    : Vesc(config, {}) {}

Vesc::Vesc(const Config& config, VescConfig vesc_config)
    : Motor(apply_defaults(config, DEFAULT_CONFIG)),
      vesc_config{.pole_pair = vesc_config.pole_pair ? vesc_config.pole_pair : uint8_t{7},
                  .auto_release_speed = vesc_config.auto_release_speed < 0 ? 120 * rpm : vesc_config.auto_release_speed} {
    subscribe();
}

void Vesc::update_control() {
    update_connection();
}

bool Vesc::send_can_cmd() {
    if (is_enabled) { // 不检测电机在线，因为未必开了反馈报文
        if (speed.ref == 0 && std::fabsf(speed.measure) < vesc_config.auto_release_speed) {
            return send_current_brake(0); // 自动释放电机，避免持续的高频注入声音
        } else {
            const float erpm = speed.ref / (rpm) * (float)vesc_config.pole_pair;
            return send_erpm(erpm);
        }
    } else {
        return send_current_brake(0); // 释放电机
    }
}

void Vesc::receive(const CanFrame& frame) {
    if (!frame.is_extend || frame.is_remote || frame.size != 8 || (frame.id & 0xff) != config.protocol.master_id)
        return;
    const uint32_t status = frame.id >> 8;
    if (status != CAN_PACKET_STATUS && status != CAN_PACKET_STATUS_4 && status != CAN_PACKET_STATUS_5)
        return;
    mark_received();
    const auto* data = frame.data;
    if (status == CAN_PACKET_STATUS) {
        const auto erpm_i32 = (int32_t)((uint32_t(data[0]) << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
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
        record_position(float(pid_pos_i16) / 50 * deg);
    } else if (status == CAN_PACKET_STATUS_5) {
        // const auto tachometer_i32 = (int32_t)((uint32_t(data[0]) << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
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

bool Vesc::send_erpm(const float erpm) const {
    const uint32_t ext_id = config.protocol.slave_id | (CAN_PACKET_SET_RPM << 8);
    CanFrame frame{.id = ext_id, .is_extend = true, .size = 4};
    make_can_data(frame.data, erpm, 1);
    return config.protocol.can.send(frame);
}

bool Vesc::send_current_brake(const float current) const {
    const uint32_t ext_id = config.protocol.slave_id | (CAN_PACKET_SET_CURRENT_BRAKE << 8);
    CanFrame frame{.id = ext_id, .is_extend = true, .size = 4};
    make_can_data(frame.data, current, 1000);
    return config.protocol.can.send(frame);
}
