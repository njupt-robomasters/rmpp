#include "aim.hpp"
#include "bsp/irq_lock.hpp"
#include <bit>
#include <cstdint>
#include <cstring>

static uint16_t crc16_modbus(const uint8_t* data, const std::size_t size) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0);
    }
    return crc;
}

static void write_float(uint8_t* data, const float value) {
    const uint32_t bits = std::bit_cast<uint32_t>(value);
    data[0] = static_cast<uint8_t>(bits);
    data[1] = static_cast<uint8_t>(bits >> 8);
    data[2] = static_cast<uint8_t>(bits >> 16);
    data[3] = static_cast<uint8_t>(bits >> 24);
}

static float read_float(const uint8_t* data) {
    const uint32_t bits = static_cast<uint32_t>(data[0]) |
                          (static_cast<uint32_t>(data[1]) << 8) |
                          (static_cast<uint32_t>(data[2]) << 16) |
                          (static_cast<uint32_t>(data[3]) << 24);
    return std::bit_cast<float>(bits);
}

Aim::Aim(const Cdc cdc, const Config& config)
    : cdc(cdc), config(config) {
    cdc.add_rx_callback([this](const uint8_t* data, const std::size_t size) { callback(data, size); });
}

void Aim::set_referee(const bool is_red, const float bullet_speed) {
    referee.is_red = is_red;
    referee.bullet_speed = std::isfinite(bullet_speed) && bullet_speed >= 0 ? bullet_speed : 0;
}

void Aim::update(const Angle yaw, const Angle pitch, const Angle roll) {
    // USB 回调只写 pending，主循环读取同一帧的目标与开火许可
    {
        const IrqLock lock;
        is_connect_vision = has_vision && callback_timer.elapsed() <= config.timeout;
        if (is_connect_vision) {
            vision = pending;
        } else {
            vision = {};
            if (has_vision) {
                has_vision = false;
                rx_size = 0;
            }
        }
    }

    if (send_timer.elapsed() < SEND_PERIOD)
        return;
    send_timer.restart();
    if (!std::isfinite(float(yaw)) || !std::isfinite(float(pitch)) || !std::isfinite(float(roll)))
        return;

    // ZYX pitch 右手正方向为低头，线上协议为抬头正；roll 沿枪口由后向前看为顺时针正
    uint8_t frame[TX_FRAME_SIZE]{0xA5, 0x5A};
    write_float(frame + 2, float(yaw) / deg);
    write_float(frame + 6, -float(pitch) / deg);
    write_float(frame + 10, float(roll) / deg);
    frame[14] = referee.is_red;
    write_float(frame + 15, referee.bullet_speed);
    const uint16_t crc = crc16_modbus(frame + 2, 17);
    frame[19] = static_cast<uint8_t>(crc);
    frame[20] = static_cast<uint8_t>(crc >> 8);
    cdc.send(frame, sizeof(frame));
}

void Aim::callback(const uint8_t* data, const std::size_t size) {
    if (has_vision && callback_timer.elapsed() > config.timeout) {
        has_vision = false;
        rx_size = 0;
    }

    for (std::size_t i = 0; i < size; i++) {
        rx_buf[rx_size++] = data[i];
        while (rx_size >= 2) {
            if (rx_buf[0] != 0xA5 || rx_buf[1] != 0x5B) {
                std::memmove(rx_buf, rx_buf + 1, --rx_size);
                continue;
            }
            if (rx_size < RX_FRAME_SIZE)
                break;

            const uint16_t crc = rx_buf[16] | (static_cast<uint16_t>(rx_buf[17]) << 8);
            const float yaw = read_float(rx_buf + 3);
            const float pitch = read_float(rx_buf + 7);
            const float distance = read_float(rx_buf + 12);
            if (crc16_modbus(rx_buf + 2, 14) != crc || rx_buf[2] > 1 || rx_buf[11] > 1 ||
                !std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(distance) || distance < 0) {
                std::memmove(rx_buf, rx_buf + 1, --rx_size);
                continue;
            }

            pending = {};
            if (rx_buf[2] == 1) {
                pending = {
                    .is_detect = true,
                    .yaw = yaw * deg,
                    .pitch = -pitch * deg,
                    .is_fire = rx_buf[11] == 1,
                    .distance = distance,
                };
            }
            has_vision = true;
            callback_timer.restart();
            rx_size = 0;
        }
    }
}
