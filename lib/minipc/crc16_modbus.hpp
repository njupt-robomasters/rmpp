#pragma once

#include <cstddef>
#include <cstdint>

// MiniPC 收发 CRC16/MODBUS：多项式 0x8005 反射为 0xA001，初值 0xFFFF，无结果异或
inline uint16_t crc16_modbus(const uint8_t* data, const size_t size) {
    uint16_t crc = 0xFFFFu;
    for (size_t index = 0; index < size; index++) {
        crc ^= data[index];
        for (size_t bit = 0; bit < 8u; bit++)
            crc = (crc >> 1u) ^ ((crc & 1u) ? 0xA001u : 0u);
    }
    return crc;
}
