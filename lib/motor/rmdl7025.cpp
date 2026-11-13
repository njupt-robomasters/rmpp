#include "rmdl7025.hpp"
#include "math/unit.hpp"

RMDL7025::RMDL7025(const Config& config)
    : Motor(config, {.max_current = MAX_CURRENT, .max_torque = MAX_TORQUE, .reduction = REDUCTION}) {
    config.bus.can.add_rx_callback([this](const CanFrame& frame) { can_callback(frame); });
}

void RMDL7025::send() {
    // 奇数次读取单圈角度
    if (send_count_++ % 2 != 0) {
        config.bus.can.send({.id = config.bus.tx_id, .size = 8, .data = {0x94}});
        return;
    }

    // 偶数次发送转矩闭环，电流 0.01 A/LSB；离线或未使能时发零电流，仍可取得应答
    float current = 0;
    if (is_connect && is_enable_)
        current = config.install.is_invert ? -current_ref : current_ref;
    const auto command = static_cast<int16_t>(current * 100.0f);
    config.bus.can.send({
        .id = config.bus.tx_id,
        .size = 8,
        .data = {0xa1, 0, 0, 0, static_cast<uint8_t>(command), static_cast<uint8_t>(command >> 8)},
    });
}

void RMDL7025::can_callback(const CanFrame& frame) {
    // 检查id和size
    if (frame.id != config.bus.rx_id || frame.size != 8)
        return;

    // 解析can报文，小端序
    const auto* data = frame.data;
    if (data[0] == 0xa1) {
        // 温度 1 ℃/LSB，电流 0.01 A/LSB，转速 1 dps/LSB
        temperature = static_cast<int8_t>(data[1]);
        raw.current = static_cast<int16_t>(data[2] | (data[3] << 8)) * 0.01f;
        raw.torque = raw.current / config.drive.max_current * config.drive.max_torque;
        raw.speed = static_cast<int16_t>(data[4] | (data[5] << 8)) * deg_s;
    } else if (data[0] == 0x94) {
        // 单圈角度 0.01°/LSB，0~35999
        raw.position = static_cast<uint16_t>(data[6] | (data[7] << 8)) * 0.01f * deg;
    } else {
        return;
    }

    // 调用上级回调
    Motor::can_callback();
}
