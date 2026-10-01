/*
 * CubeMX 配置：Connectivity → SPI1，APB2 = 84 MHz 时 SPI 时钟为 10.5 MHz
 *
 * SPI 参数
 * | 页面                         | 参数                                       | 设置                         |
 * | ---------------------------- | ------------------------------------------ | ---------------------------- |
 * | Mode                         | Mode / Hardware NSS Signal                 | Full-Duplex Master / Disable |
 * | Parameter Settings           | Data Size / First Bit                      | 8 Bits / MSB First           |
 * | Parameter Settings           | Clock Polarity (CPOL) / Clock Phase (CPHA) | High / 2 Edge                |
 * | Parameter Settings           | Prescaler (for Baud Rate)                  | 8                            |
 * | Parameter Settings           | TI Mode / CRC Calculation                  | Disable / Disable            |
 * | DMA Settings / NVIC Settings | DMA / SPI1 global interrupt                | 不添加 DMA，不勾选 SPI 中断  |
 * | System Core → SYS            | Debug                                      | Serial Wire，释放 PB3/PB4    |
 * | Pinout                       | SPI1_SCK / SPI1_MISO / SPI1_MOSI           | PB3 / PB4 / PA7              |
 *
 * System Core → GPIO，两个片选均设为 GPIO_Output
 * | 引脚 | User Label | GPIO mode        | GPIO output level | GPIO Pull-up/Pull-down |
 * | ---- | ---------- | ---------------- | ----------------- | ---------------------- |
 * | PA4  | CS1_ACCEL  | Output Push Pull | High              | Pull-up                |
 * | PB0  | CS1_GYRO   | Output Push Pull | High              | Pull-up                |
 *
 * 两个片选的 Maximum output speed 均为 High；生成 CS1_ACCEL / CS1_GYRO 的 Pin 和 GPIO_Port 宏
 */

#include "bsp/spi.hpp"
#include "spi.h"

struct Spi::Context {
    int bus;
    int chip_select;
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* port;
    uint16_t pin;
};

Spi::Spi(const int bus, const int chip_select) {
    // 同一总线上的两个独立片选：BMI088 加速度计、陀螺仪
    static Context contexts[] = {
        {.bus = 1, .chip_select = 1, .hspi = &hspi1, .port = CS1_ACCEL_GPIO_Port, .pin = CS1_ACCEL_Pin},
        {.bus = 1, .chip_select = 2, .hspi = &hspi1, .port = CS1_GYRO_GPIO_Port, .pin = CS1_GYRO_Pin},
    };
    for (auto& context : contexts) {
        if (context.bus == bus && context.chip_select == chip_select) {
            context_ = &context;
            break;
        }
    }
}

void Spi::transfer(const uint8_t* tx_buf, uint8_t* rx_buf, std::size_t size) const {
    if (!context_)
        return;

    // 同步事务仅在普通上下文调用，整个传输期间保持中断可用
    HAL_GPIO_WritePin(context_->port, context_->pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(context_->hspi, tx_buf, rx_buf, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(context_->port, context_->pin, GPIO_PIN_SET);
}
