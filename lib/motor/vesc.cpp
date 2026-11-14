#include "vesc.hpp"
#include "math/unit.hpp"

Vesc::Vesc(const Motor::Config& config)
    : Motor(config, 0, 0, 0) {
    config.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void Vesc::set_current(const float current) {
    Motor::set_current(current);
    const float direction = is_invert ? -1.0f : 1.0f;
    set_int32(CAN_PACKET_SET_CURRENT, encode32(direction * current_ref, 1000));
}

void Vesc::set_torque(const float torque) {
    Motor::set_torque(torque);
    const float direction = is_invert ? -1.0f : 1.0f;
    set_int32(CAN_PACKET_SET_CURRENT, encode32(direction * current_ref, 1000));
}

void Vesc::set_duty(const float duty) {
    set_int32(CAN_PACKET_SET_DUTY, encode32(duty, 100000));
}

void Vesc::set_current_brake(const float current) {
    set_int32(CAN_PACKET_SET_CURRENT_BRAKE, encode32(current, 1000));
}

void Vesc::set_speed(const float speed) {
    const float direction = is_invert ? -1.0f : 1.0f;
    set_int32(CAN_PACKET_SET_RPM, encode32(direction * speed / rpm * reduction, 1));
}

void Vesc::set_position(const float position) {
    const float direction = is_invert ? -1.0f : 1.0f;
    set_int32(CAN_PACKET_SET_POS, encode32(direction * position / deg * reduction, 1000000));
}

void Vesc::set_current_relative(const float current) {
    set_int32(CAN_PACKET_SET_CURRENT_REL, encode32(current, 100000));
}

void Vesc::set_current_brake_relative(const float current) {
    set_int32(CAN_PACKET_SET_CURRENT_BRAKE_REL, encode32(current, 100000));
}

void Vesc::set_handbrake(const float current) {
    set_int32(CAN_PACKET_SET_CURRENT_HANDBRAKE, encode32(current, 1000));
}

void Vesc::set_handbrake_relative(const float current) {
    set_int32(CAN_PACKET_SET_CURRENT_HANDBRAKE_REL, encode32(current, 100000));
}

void Vesc::set_current_off_delay(const float current, const float delay) {
    set_int16(CAN_PACKET_SET_CURRENT, encode32(current, 1000), encode16(delay, 1000));
}

void Vesc::set_current_relative_off_delay(const float current, const float delay) {
    set_int16(CAN_PACKET_SET_CURRENT_REL, encode32(current, 100000), encode16(delay, 1000));
}

void Vesc::send() const {
    Command command = this->command;
    uint8_t size = command_size;
    uint8_t data[8] = {command_data[0], command_data[1], command_data[2], command_data[3],
                       command_data[4], command_data[5], command_data[6], command_data[7]};
    if (!is_enable) {
        command = CAN_PACKET_SET_CURRENT;
        size = 4;
        const int32_t current = 0;
        data[0] = static_cast<uint8_t>(current >> 24);
        data[1] = static_cast<uint8_t>(current >> 16);
        data[2] = static_cast<uint8_t>(current >> 8);
        data[3] = static_cast<uint8_t>(current);
    }
    can.send({.id = tx_id | (static_cast<uint32_t>(command) << 8),
              .is_extend = true,
              .size = size,
              .data = {data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]}});
}

void Vesc::can_callback(const CanFrame& frame) {
    if ((frame.id & 0xff) != rx_id || frame.size != 8)
        return;

    const auto status = static_cast<Status>(frame.id >> 8);
    const auto* data = frame.data;
    switch (status) {
    case CAN_PACKET_STATUS:
        raw_speed = static_cast<float>(read32(data)) * rpm;
        raw_current = static_cast<float>(read16(data + 4)) / 10.0f;
        raw_torque = raw_current / max_current * max_torque;
        duty_cycle = static_cast<float>(read16(data + 6)) / 1000.0f * pct;
        break;
    case CAN_PACKET_STATUS_2:
        amp_hours = static_cast<float>(read32(data)) / 10000.0f;
        amp_hours_charged = static_cast<float>(read32(data + 4)) / 10000.0f;
        break;
    case CAN_PACKET_STATUS_3:
        watt_hours = static_cast<float>(read32(data)) / 10000.0f;
        watt_hours_charged = static_cast<float>(read32(data + 4)) / 10000.0f;
        break;
    case CAN_PACKET_STATUS_4:
        temp_fet = static_cast<float>(read16(data)) / 10.0f;
        temp_motor = static_cast<float>(read16(data + 2)) / 10.0f;
        current_in = static_cast<float>(read16(data + 4)) / 10.0f;
        pid_pos = static_cast<float>(read16(data + 6)) / 50.0f * deg;
        break;
    case CAN_PACKET_STATUS_5:
        tachometer = static_cast<float>(read32(data)) / 6.0f * rev;
        voltage_in = static_cast<float>(read16(data + 4)) / 10.0f;
        break;
    case CAN_PACKET_STATUS_6:
        adc1 = static_cast<float>(read16(data)) / 1000.0f;
        adc2 = static_cast<float>(read16(data + 2)) / 1000.0f;
        adc3 = static_cast<float>(read16(data + 4)) / 1000.0f;
        ppm = static_cast<float>(read16(data + 6)) / 1000.0f * pct;
        break;
    default:
        return;
    }
    Motor::can_callback();
}

void Vesc::set_int32(const Command command, const int32_t value) {
    this->command = command;
    command_size = 4;
    command_data[0] = static_cast<uint8_t>(value >> 24);
    command_data[1] = static_cast<uint8_t>(value >> 16);
    command_data[2] = static_cast<uint8_t>(value >> 8);
    command_data[3] = static_cast<uint8_t>(value);
}

void Vesc::set_int16(const Command command, const int32_t first, const int16_t second) {
    this->command = command;
    command_size = 6;
    command_data[0] = static_cast<uint8_t>(first >> 24);
    command_data[1] = static_cast<uint8_t>(first >> 16);
    command_data[2] = static_cast<uint8_t>(first >> 8);
    command_data[3] = static_cast<uint8_t>(first);
    command_data[4] = static_cast<uint8_t>(second >> 8);
    command_data[5] = static_cast<uint8_t>(second);
}

int16_t Vesc::encode16(const float value, const float scale) {
    return static_cast<int16_t>(value * scale);
}

int32_t Vesc::encode32(const float value, const float scale) {
    return static_cast<int32_t>(value * scale);
}

int16_t Vesc::read16(const uint8_t* data) {
    const uint16_t value = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
    return static_cast<int16_t>(value);
}

int32_t Vesc::read32(const uint8_t* data) {
    const uint32_t value = (static_cast<uint32_t>(data[0]) << 24) |
                           (static_cast<uint32_t>(data[1]) << 16) |
                           (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
    return static_cast<int32_t>(value);
}
