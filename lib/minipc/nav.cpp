#include "nav.hpp"
#include "crc16_modbus.hpp"
#include <cstddef>

Nav::Nav(const Config& config)
    : config(config) {
    config.cdc.add_rx_callback([this](const uint8_t* data, const size_t size) { callback(data, size); });
}

void Nav::update() {
    // 断联检测：指令复位，原始帧保留
    if (callback_timer.elapsed() > config.timeout) {
        is_connect = false;
        vx = vy = yaw = 0;
        action_flag = 0;
    }
}

void Nav::callback(const uint8_t* data, const size_t size) {
    for (size_t index = 0; index < size; index++) {
        rx_fifo.push(data[index]);
        while (rx_fifo.size >= 2) {
            // 帧头不符：丢弃 1 字节重新对齐
            if (rx_fifo[0] != 0xA5 || rx_fifo[1] != 0x5A) {
                rx_fifo.pop(1);
                continue;
            }
            if (rx_fifo.size < sizeof(raw))
                break;

            // CRC 不符：丢弃 1 字节，帧内出现的帧头可重新对齐
            RxFrame frame;
            auto* bytes = reinterpret_cast<uint8_t*>(&frame);
            rx_fifo.peek(bytes, sizeof(frame));
            if (crc16_modbus(bytes + sizeof(frame.sof), offsetof(RxFrame, crc) - sizeof(frame.sof)) != frame.crc) {
                rx_fifo.pop(1);
                continue;
            }
            rx_fifo.pop(sizeof(frame));

            // 有效帧：保存原始帧并解析
            raw = frame;
            vx = raw.vx;
            vy = raw.vy;
            yaw = raw.yaw;
            action_flag = raw.action_flag;
            is_connect = true;
            callback_timer.restart();
        }
    }
}
