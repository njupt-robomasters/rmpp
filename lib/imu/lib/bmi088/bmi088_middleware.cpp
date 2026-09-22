#include "bmi088_middleware.h"
#include "bsp/timer.hpp"
#include "math/unit.hpp"
#include <array>
#include <cstring>

// 原有 BMI088 驱动使用一个实例；绑定只在 Imu 初始化时进行。
static Spi accel_spi;
static Spi gyro_spi;

void bmi088_bind(Spi accel, Spi gyro) {
    accel_spi = accel;
    gyro_spi = gyro;
}

void bmi088_delay_ms(uint32_t milliseconds) {
    Timer::delay(milliseconds * ms);
}

static void write_reg(Spi spi, uint8_t reg, uint8_t value) {
    const std::array<uint8_t, 2> tx{reg, value};
    std::array<uint8_t, 2> rx{};
    spi.transfer({.tx = tx.data(), .rx = rx.data(), .size = tx.size()});
}

static void read_regs(Spi spi, uint8_t reg, uint8_t* data, uint8_t size, uint8_t prefix, uint8_t dummy) {
    // 最长读取为陀螺仪的 8 字节，额外预留地址和空读字节。
    if (!data || size == 0 || size > 8)
        return;
    std::array<uint8_t, 10> tx{}, rx{};
    tx.fill(0x55);
    tx[0] = reg | 0x80;
    if (prefix == 2)
        tx[1] = dummy;
    if (spi.transfer({.tx = tx.data(), .rx = rx.data(), .size = static_cast<std::size_t>(size + prefix)}))
        std::memcpy(data, rx.data() + prefix, size);
    else
        std::memset(data, 0, size);
}

void bmi088_accel_write_reg(uint8_t reg, uint8_t value) {
    write_reg(accel_spi, reg, value);
}

void bmi088_gyro_write_reg(uint8_t reg, uint8_t value) {
    write_reg(gyro_spi, reg, value);
}

uint8_t bmi088_accel_read_reg(uint8_t reg) {
    uint8_t value = 0;
    read_regs(accel_spi, reg, &value, 1, 2, 0x55);
    return value;
}

uint8_t bmi088_gyro_read_reg(uint8_t reg) {
    uint8_t value = 0;
    read_regs(gyro_spi, reg, &value, 1, 1, 0x55);
    return value;
}

void bmi088_accel_read_regs(uint8_t reg, uint8_t* data, uint8_t size) {
    read_regs(accel_spi, reg, data, size, 2, reg | 0x80);
}

void bmi088_gyro_read_regs(uint8_t reg, uint8_t* data, uint8_t size) {
    read_regs(gyro_spi, reg, data, size, 1, 0x55);
}
