#include "bmi088.hpp"
#include "bsp/timer.hpp"
#include <cstring>

Bmi088::Bmi088(const Config& config) : config(config) {}

bool Bmi088::init() const {
    uint8_t id = 0;

    // 初始化加速度计
    // 上电及复位后的第一次读取用于切换到 SPI 模式，因此这里连续读两次
    read_registers(config.accel, 0x00, &id, 1, 2);
    Timer::delay(0.001f);
    read_registers(config.accel, 0x00, &id, 1, 2);
    Timer::delay(0.001f);
    write_register(config.accel, 0x7E, 0xB6);
    Timer::delay(0.080f);
    read_registers(config.accel, 0x00, &id, 1, 2);
    Timer::delay(0.001f);
    read_registers(config.accel, 0x00, &id, 1, 2);
    Timer::delay(0.001f);
    if (id != 0x1E)
        return false;

    // 寄存器地址、写入值；轮询读取不启用数据就绪中断
    static constexpr uint8_t ACCEL_REGISTERS[][2] = {
        {0x7D, 0x04}, // 加速度计使能
        {0x7C, 0x00}, // 正常功耗
        {0x40, 0xAB}, // 800 Hz，normal 带宽
        {0x41, 0x01}, // ±6 g
    };
    for (const auto& reg : ACCEL_REGISTERS) {
        // 写寄存器
        write_register(config.accel, reg[0], reg[1]);
        Timer::delay(0.001f);

        // 读出检验
        uint8_t value = 0;
        read_registers(config.accel, reg[0], &value, 1, 2);
        Timer::delay(0.001f);
        if (value != reg[1])
            return false;
    }

    // 初始化角速度计
    read_registers(config.gyro, 0x00, &id, 1, 1);
    Timer::delay(0.001f);
    read_registers(config.gyro, 0x00, &id, 1, 1);
    Timer::delay(0.001f);
    write_register(config.gyro, 0x14, 0xB6);
    Timer::delay(0.080f);
    read_registers(config.gyro, 0x00, &id, 1, 1);
    Timer::delay(0.001f);
    read_registers(config.gyro, 0x00, &id, 1, 1);
    Timer::delay(0.001f);
    if (id != 0x0F)
        return false;

    static constexpr uint8_t GYRO_REGISTERS[][2] = {
        {0x0F, 0x00}, // ±2000 °/s
        {0x10, 0x81}, // 2000 Hz，230 Hz 带宽
        {0x11, 0x00}, // 正常功耗
    };
    for (const auto& reg : GYRO_REGISTERS) {
        // 写寄存器
        write_register(config.gyro, reg[0], reg[1]);
        Timer::delay(0.001f);

        // 读出检验
        uint8_t value = 0;
        read_registers(config.gyro, reg[0], &value, 1, 1);
        Timer::delay(0.001f);
        if (value != reg[1])
            return false;
    }

    return true;
}

bool Bmi088::update() {
    uint8_t accel_buf[6]{}, gyro_buf[8]{}, temp_buf[2]{};
    read_registers(config.accel, 0x12, accel_buf, 6, 2);
    read_registers(config.gyro, 0x00, gyro_buf, 8, 1);
    read_registers(config.accel, 0x22, temp_buf, 2, 2);
    if (gyro_buf[0] != 0x0F)
        return false;

    for (int i = 0; i < 3; ++i) {
        const auto raw_accel = static_cast<int16_t>(accel_buf[2 * i] | (accel_buf[2 * i + 1] << 8));
        const auto raw_gyro = static_cast<int16_t>(gyro_buf[2 * i + 2] | (gyro_buf[2 * i + 3] << 8));
        accel[i] = raw_accel * 0.00179443359375f;
        gyro[i] = raw_gyro * 0.0010652644360316953f;
    }

    int raw_temperature = (temp_buf[0] << 3) | (temp_buf[1] >> 5);
    if (raw_temperature > 1023)
        raw_temperature -= 2048;
    temperature = raw_temperature * 0.125f + 23.0f;

    return true;
}

void Bmi088::write_register(const Spi& spi, const uint8_t reg, const uint8_t value) {
    const uint8_t tx_buf[2]{reg, value};
    uint8_t rx_buf[2]{};
    spi.transfer(tx_buf, rx_buf, sizeof(tx_buf));
}

void Bmi088::read_registers(const Spi& spi, const uint8_t reg, uint8_t* data, const uint8_t size, const uint8_t prefix) {
    uint8_t tx_buf[10]{}, rx_buf[10]{};
    tx_buf[0] = reg | 0x80;
    spi.transfer(tx_buf, rx_buf, size + prefix);
    std::memcpy(data, rx_buf + prefix, size);
}
