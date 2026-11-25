#include "aim.hpp"
#include "crc16_modbus.hpp"
#include <cstddef>

Aim::Aim(const Config& config)
    : config(config) {
    config.cdc.add_rx_callback([this](const uint8_t* data, const size_t size) { callback(data, size); });
}

void Aim::set_attitude(const Quaternionf& quaternion) {
    raw_tx.qx = quaternion.x();
    raw_tx.qy = quaternion.y();
    raw_tx.qz = quaternion.z();
    raw_tx.qw = quaternion.w();
}

void Aim::set_referee(const bool is_red, const float bullet_speed) {
    raw_tx.is_red = is_red;
    raw_tx.bullet_speed = bullet_speed;
}

void Aim::send() {
    auto* bytes = reinterpret_cast<uint8_t*>(&raw_tx);
    raw_tx.crc = crc16_modbus(bytes, offsetof(TxFrame, crc));
    config.cdc.send(bytes, sizeof(raw_tx));
}

void Aim::update() {
    // 断联检测：目标复位，原始帧保留
    if (callback_timer.elapsed() > config.timeout) {
        is_connect = false;
        is_detect = is_fire = false;
        yaw = pitch = 0;
        distance = 0;
    }
}

void Aim::callback(const uint8_t* data, const size_t size) {
    for (size_t index = 0; index < size; index++) {
        rx_fifo.push(data[index]);
        while (rx_fifo.size >= 1) {
            // 帧头不符：丢弃 1 字节重新对齐
            if (rx_fifo[0] != SOF_RX) {
                rx_fifo.pop(1);
                continue;
            }
            if (rx_fifo.size < sizeof(raw_rx))
                break;

            // CRC 不符：丢弃 1 字节，帧内出现的帧头可重新对齐
            RxFrame frame;
            auto* bytes = reinterpret_cast<uint8_t*>(&frame);
            rx_fifo.peek(bytes, sizeof(frame));
            if (crc16_modbus(bytes, offsetof(RxFrame, crc)) != frame.crc) {
                rx_fifo.pop(1);
                continue;
            }
            rx_fifo.pop(sizeof(frame));

            // 有效帧：保存原始帧并解析，线上 pitch 向下为正，与内部同号
            raw_rx = frame;
            is_detect = raw_rx.is_detect;
            yaw = raw_rx.yaw * deg;
            pitch = raw_rx.pitch * deg;
            is_fire = raw_rx.is_fire;
            distance = raw_rx.distance;
            is_connect = true;
            callback_timer.restart();
        }
    }
}
