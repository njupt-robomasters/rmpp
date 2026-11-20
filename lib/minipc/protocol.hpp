#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>

// MiniPC 公共编码：IEEE 754 float32 小端，CRC-16/MODBUS
static inline uint16_t minipc_crc16_modbus(const uint8_t* data, const std::size_t size) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0);
    }
    return crc;
}

static inline float minipc_read_float(const uint8_t* data) {
    const uint32_t bits = static_cast<uint32_t>(data[0]) |
                          (static_cast<uint32_t>(data[1]) << 8) |
                          (static_cast<uint32_t>(data[2]) << 16) |
                          (static_cast<uint32_t>(data[3]) << 24);
    return std::bit_cast<float>(bits);
}

static inline void minipc_write_float(uint8_t* data, const float value) {
    const uint32_t bits = std::bit_cast<uint32_t>(value);
    for (int i = 0; i < 4; i++)
        data[i] = static_cast<uint8_t>(bits >> (8 * i));
}
