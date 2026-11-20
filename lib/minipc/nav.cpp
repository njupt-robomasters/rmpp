#include "nav.hpp"
#include "bsp/irq_lock.hpp"
#include <bit>
#include <cmath>
#include <cstring>

Nav::Nav(const Cdc cdc, const Config& config)
    : config(config) {
    cdc.add_rx_callback([this](const uint8_t* data, const std::size_t size) { callback(data, size); });
}

void Nav::update() {
    // USB 回调与主循环共享待发布指令及计时器，短临界区保证整帧一致
    const IrqLock lock;
    is_connect = has_command && callback_timer.elapsed() <= config.timeout;
    if (is_connect) {
        command = pending;
    } else {
        command = {};
        if (has_command) {
            has_command = false;
            rx_size = 0;
        }
    }
}

void Nav::callback(const uint8_t* data, const std::size_t size) {
    // 超时前的半帧丢弃，新连接允许从任意分片开始接收
    if (has_command && callback_timer.elapsed() > config.timeout) {
        has_command = false;
        rx_size = 0;
    }

    for (std::size_t i = 0; i < size; i++) {
        rx_buf[rx_size++] = data[i];
        while (rx_size >= 2) {
            if (rx_buf[0] != 0xA5 || rx_buf[1] != 0x5A) {
                std::memmove(rx_buf, rx_buf + 1, --rx_size);
                continue;
            }
            if (rx_size < FRAME_SIZE)
                break;

            const uint16_t crc = rx_buf[15] | (static_cast<uint16_t>(rx_buf[16]) << 8);
            const Command received{
                .vx = read_float(rx_buf + 2),
                .vy = read_float(rx_buf + 6),
                .yaw = read_float(rx_buf + 10),
                .action_flag = rx_buf[14],
            };
            if (crc16_modbus(rx_buf + 2, 13) != crc ||
                !std::isfinite(received.vx) || !std::isfinite(received.vy) || !std::isfinite(received.yaw) ||
                received.action_flag > 1) {
                std::memmove(rx_buf, rx_buf + 1, --rx_size);
                continue;
            }

            pending = received;
            has_command = true;
            callback_timer.restart();
            rx_size = 0;
        }
    }
}

uint16_t Nav::crc16_modbus(const uint8_t* data, const std::size_t size) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0);
    }
    return crc;
}

float Nav::read_float(const uint8_t* data) {
    const uint32_t bits = static_cast<uint32_t>(data[0]) |
                          (static_cast<uint32_t>(data[1]) << 8) |
                          (static_cast<uint32_t>(data[2]) << 16) |
                          (static_cast<uint32_t>(data[3]) << 24);
    return std::bit_cast<float>(bits);
}
