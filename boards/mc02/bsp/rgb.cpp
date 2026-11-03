/*
 * WS2812：SPI6 MOSI / PA7，只发送；SPI6 时钟 HSE 24 MHz，4 分频得 6 MHz，每位 1.33 μs
 * 每个数据位编码为 1 字节：0 → 0x60（高 333 ns），1 → 0x7C（高 833 ns）；GRB 顺序，高位先发
 * Keep IO State 开启时 MOSI 始终推挽；HAL 发完关闭 SPE 后，实测 MOSI 变为 FIFO 中某个已发字节的首位
 * 编码字节首尾位均为 0，发完线路保持低电平即为锁存复位
 * 两次调用间隔须大于 280 μs；阻塞发送约 32 μs
 */

#include "bsp/rgb.hpp"
#include "spi.h"
#include <algorithm>

void Rgb::set(const float red, const float green, const float blue) const {
    uint8_t buf[24];
    const float colors[3] = {green, red, blue};
    for (int i = 0; i < 3; i++) {
        const auto level = static_cast<uint8_t>(std::clamp(colors[i], 0.0f, 1.0f) * 255.0f + 0.5f);
        for (int bit = 0; bit < 8; bit++)
            buf[8 * i + bit] = (level >> (7 - bit)) & 1 ? 0x7C : 0x60;
    }
    HAL_SPI_Transmit(&hspi6, buf, sizeof(buf), HAL_MAX_DELAY);
}
