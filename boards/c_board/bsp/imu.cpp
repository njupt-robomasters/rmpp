/*
 * BMI088：Connectivity → SPI1，APB2 = 84 MHz 时 SPI 时钟为 10.5 MHz
 * | 页面                         | 参数                                       | 设置                         |
 * | ---------------------------- | ------------------------------------------ | ---------------------------- |
 * | Mode                         | Mode / Hardware NSS Signal                 | Full-Duplex Master / Disable |
 * | Parameter Settings           | Data Size / First Bit                      | 8 Bits / MSB First           |
 * | Parameter Settings           | Clock Polarity (CPOL) / Clock Phase (CPHA) | High / 2 Edge                |
 * | Parameter Settings           | Prescaler (for Baud Rate)                  | 8                            |
 * | DMA Settings / NVIC Settings | DMA / SPI1 global interrupt                | 不添加 DMA，不勾选 SPI 中断  |
 * | System Core → SYS            | Debug                                      | Serial Wire，释放 PB3/PB4    |
 * | Pinout                       | SPI1_SCK / SPI1_MISO / SPI1_MOSI           | PB3 / PB4 / PA7              |
 *
 * 片选：PA4 = SPI1_CS_ACC、PB0 = SPI1_CS_GYRO，GPIO_Output，推挽，初始高电平
 * 加热：Timers → TIM10，CH1 设为 PWM Generation，引脚 PF6，PWM mode 1，CH Polarity High，Pulse 0
 */

#include "bsp/imu.hpp"
#include "bsp/timer.hpp"
#include "spi.h"
#include "tim.h"
#include <algorithm>
#include <cstring>

struct Chip {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t prefix; // 读取时地址后的字节数：加速度计多一个空读字节
};

static constexpr Chip ACCEL = {SPI1_CS_ACC_GPIO_Port, SPI1_CS_ACC_Pin, 2};
static constexpr Chip GYRO = {SPI1_CS_GYRO_GPIO_Port, SPI1_CS_GYRO_Pin, 1};

// 同步事务仅在普通上下文调用，传输期间保持中断可用
static void transfer(const Chip& chip, const uint8_t* tx_buf, uint8_t* rx_buf, const uint16_t size) {
    HAL_GPIO_WritePin(chip.port, chip.pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(chip.port, chip.pin, GPIO_PIN_SET);
}

static void write_register(const Chip& chip, const uint8_t reg, const uint8_t value) {
    const uint8_t tx_buf[2]{reg, value};
    uint8_t rx_buf[2]{};
    transfer(chip, tx_buf, rx_buf, sizeof(tx_buf));
}

// 最多读取 8 字节
static void read_registers(const Chip& chip, const uint8_t reg, uint8_t* data, const uint8_t size) {
    uint8_t tx_buf[10]{}, rx_buf[10]{};
    tx_buf[0] = reg | 0x80;
    transfer(chip, tx_buf, rx_buf, size + chip.prefix);
    std::memcpy(data, rx_buf + chip.prefix, size);
}

// 软复位并核对芯片 ID；上电及复位后的第一次读取用于切换到 SPI 模式，因此连续读两次
static bool reset(const Chip& chip, const uint8_t reset_reg, const uint8_t id) {
    uint8_t value = 0;
    read_registers(chip, 0x00, &value, 1);
    Timer::delay(0.001f);
    read_registers(chip, 0x00, &value, 1);
    Timer::delay(0.001f);
    write_register(chip, reset_reg, 0xB6);
    Timer::delay(0.080f);
    read_registers(chip, 0x00, &value, 1);
    Timer::delay(0.001f);
    read_registers(chip, 0x00, &value, 1);
    Timer::delay(0.001f);
    return value == id;
}

// 依次写入寄存器并回读检验
static bool configure(const Chip& chip, const uint8_t (*registers)[2], const std::size_t count) {
    for (std::size_t i = 0; i < count; i++) {
        write_register(chip, registers[i][0], registers[i][1]);
        Timer::delay(0.001f);

        uint8_t value = 0;
        read_registers(chip, registers[i][0], &value, 1);
        Timer::delay(0.001f);
        if (value != registers[i][1])
            return false;
    }
    return true;
}

static bool init_sensor() {
    // 寄存器地址、写入值；轮询读取不启用数据就绪中断
    static constexpr uint8_t ACCEL_REGISTERS[][2] = {
        {0x7D, 0x04}, // 加速度计使能
        {0x7C, 0x00}, // 正常功耗
        {0x40, 0xAB}, // 800 Hz，normal 带宽
        {0x41, 0x01}, // ±6 g
    };
    static constexpr uint8_t GYRO_REGISTERS[][2] = {
        {0x0F, 0x00}, // ±2000 °/s
        {0x10, 0x81}, // 2000 Hz，230 Hz 带宽
        {0x11, 0x00}, // 正常功耗
    };
    return reset(ACCEL, 0x7E, 0x1E) && configure(ACCEL, ACCEL_REGISTERS, std::size(ACCEL_REGISTERS)) &&
           reset(GYRO, 0x14, 0x0F) && configure(GYRO, GYRO_REGISTERS, std::size(GYRO_REGISTERS));
}

void imu_init() {
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

void Imu::update() {
    // 初始化传感器，失败时下次重试
    if (!is_initialized_) {
        is_initialized_ = init_sensor();
        if (!is_initialized_)
            return;
    }

    // 读取加速度、角速度（含芯片 ID）和温度；失败时关闭加热并保持上次输出
    uint8_t accel_buf[6]{}, gyro_buf[8]{}, temp_buf[2]{};
    read_registers(ACCEL, 0x12, accel_buf, 6);
    read_registers(GYRO, 0x00, gyro_buf, 8);
    read_registers(ACCEL, 0x22, temp_buf, 2);
    if (gyro_buf[0] != 0x0F) {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
        return;
    }

    // 换算到国际单位：±6 g 与 ±2000 °/s 量程
    Vector3 raw_gyro, raw_accel;
    for (int i = 0; i < 3; i++) {
        raw_accel[i] = static_cast<int16_t>(accel_buf[2 * i] | (accel_buf[2 * i + 1] << 8)) * 0.00179443359375f;
        raw_gyro[i] = static_cast<int16_t>(gyro_buf[2 * i + 2] | (gyro_buf[2 * i + 3] << 8)) * 0.0010652644360316953f;
    }

    // 11 位补码，0.125 °C/LSB，零点 23 °C
    int raw_temperature = (temp_buf[0] << 3) | (temp_buf[1] >> 5);
    if (raw_temperature > 1023)
        raw_temperature -= 2048;
    temperature = raw_temperature * 0.125f + 23.0f;

    // 加热闭环：[0, 1] 占空比换算为 PWM 比较值
    const float duty = std::clamp(temperature_pid_.calculate(config_.temperature - temperature), 0.0f, 1.0f);
    temperature_pid_.track(duty);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, static_cast<uint32_t>((__HAL_TIM_GET_AUTORELOAD(&htim10) + 1) * duty));

    // 零偏校准，再修正零偏、重力模长与安装方向
    update_calibration(raw_gyro, raw_accel);
    gyro = config_.orientation * (raw_gyro - config_.gyro_offset);
    accel = config_.orientation * (raw_accel * (9.81f / config_.gravity));
}
