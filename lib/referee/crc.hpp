#pragma once

#include <cstddef>
#include <cstdint>

// 裁判系统帧头 CRC8：多项式 0x31 反射为 0x8C，初值 0xFF，无结果异或；VT13 图传链路共用
uint8_t crc8_referee(const uint8_t* data, size_t size);

// 裁判系统整帧 CRC16：多项式 0x1021 反射为 0x8408，初值 0xFFFF，无结果异或；VT13 遥控帧与图传链路共用
uint16_t crc16_referee(const uint8_t* data, size_t size);
