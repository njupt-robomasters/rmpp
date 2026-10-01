#include "vesc.hpp"
#include <cmath>

Vesc::Vesc(const Config& config)
    : Motor(config.motor), pole_pair_(config.pole_pair), auto_release_speed_(config.auto_release_speed) {
    config_.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void Vesc::send() const {
    // 不检测在线，因为未必开启反馈报文；零目标低速时释放，避免持续高频注入声音
    if (!is_enable || !std::isfinite(speed.ref) || (speed.ref == 0 && std::fabs(speed.measure) < auto_release_speed_))
        send_command(CAN_PACKET_SET_CURRENT_BRAKE, 0, 1000);
    else
        send_command(CAN_PACKET_SET_RPM, (config_.install.is_invert ? -speed.ref : speed.ref) * config_.drive.reduction / rpm * pole_pair_, 1);
}

void Vesc::can_callback(const CanFrame& frame) {
    // 扩展帧低 8 位为控制器 ID，高位为状态包编号
    if ((frame.id & 0xff) != config_.bus.rx_id || frame.size != 8)
        return;
    const uint32_t status = frame.id >> 8;
    const auto* data = frame.data;
    if (status == CAN_PACKET_STATUS) {
        const auto erpm = static_cast<int32_t>((uint32_t(data[0]) << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
        raw.speed = float(erpm) / pole_pair_ / config_.drive.reduction * rpm;
        raw.current = static_cast<int16_t>((data[4] << 8) | data[5]) / 10.0f;
        raw.torque = raw.current * config_.drive.kt;
        duty = static_cast<int16_t>((data[6] << 8) | data[7]) / 1000.0f * pct;
    } else if (status == CAN_PACKET_STATUS_4) {
        temperature.mos = static_cast<int16_t>((data[0] << 8) | data[1]) / 10.0f;
        temperature.motor = static_cast<int16_t>((data[2] << 8) | data[3]) / 10.0f;
        input.current = static_cast<int16_t>((data[4] << 8) | data[5]) / 10.0f;
        raw.position = static_cast<int16_t>((data[6] << 8) | data[7]) / 50.0f * deg;
    } else if (status == CAN_PACKET_STATUS_5) {
        input.voltage = static_cast<int16_t>((data[4] << 8) | data[5]) / 10.0f;
    } else {
        return;
    }
    Motor::can_callback();
}

void Vesc::send_command(Command command, float value, float scale) const {
    const auto number = static_cast<int32_t>(value * scale);
    config_.bus.can.send({
        .id = config_.bus.tx_id | (uint32_t(command) << 8),
        .is_extend = true,
        .size = 4,
        .data = {uint8_t(number >> 24), uint8_t(number >> 16), uint8_t(number >> 8), uint8_t(number)},
    });
}
