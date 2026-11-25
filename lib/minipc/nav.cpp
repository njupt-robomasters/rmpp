#include "nav.hpp"
#include "crc16_modbus.hpp"
#include <cstddef>

Nav::Nav(const Config& config)
    : config(config) {
    config.cdc.add_rx_callback([this](const uint8_t* data, const size_t size) { callback(data, size); });
}

void Nav::set_target(const float x, const float y) {
    raw_tx.x = x;
    raw_tx.y = y;
}

void Nav::send(const float yaw_ecd) {
    raw_tx.yaw = yaw_ecd;
    auto* bytes = reinterpret_cast<uint8_t*>(&raw_tx);
    raw_tx.crc = crc16_modbus(bytes, offsetof(TxFrame, crc));
    config.cdc.send(bytes, sizeof(raw_tx));
}

void Nav::update() {
    // 断联检测：指令复位，原始帧保留
    if (callback_timer.elapsed() > config.timeout) {
        is_connect = false;
        is_through_hole = false;
        vx = vy = yaw = 0;
        x = y = 0;
    }
}

void Nav::callback(const uint8_t* data, const size_t size) {
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

            // 有效帧：保存原始帧并解析
            raw_rx = frame;
            is_through_hole = raw_rx.is_through_hole;
            vx = raw_rx.vx;
            vy = raw_rx.vy;
            yaw = raw_rx.yaw;
            x = raw_rx.x;
            y = raw_rx.y;
            is_connect = true;
            callback_timer.restart();
        }
    }
}
