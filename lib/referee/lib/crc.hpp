#pragma once

#include <cstdint>

class CRC8 {
  public:
    static uint8_t check_sum(const uint8_t data[], uint32_t len);

    static bool verify(const uint8_t data[], uint32_t len, uint8_t crc8);

  private:
    static const uint8_t TABLE[256];
};

class CRC16 {
  public:
    static uint16_t check_sum(const uint8_t data[], uint32_t len);

    static bool verify(const uint8_t data[], uint32_t len, uint16_t crc16);

  private:
    static const uint16_t TABLE[256];
};