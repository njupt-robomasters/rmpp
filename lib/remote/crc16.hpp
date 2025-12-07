#pragma once

#include <cstdint>

class CRC16 {
public:
    static uint16_t CheckSum(const uint8_t data[], uint32_t len);

    static bool Verify(const uint8_t data[], uint32_t len, uint16_t crc16);

private:
    static const uint16_t CRC16_TABLE[256];
};