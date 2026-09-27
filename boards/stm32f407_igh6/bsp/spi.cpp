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
#include <limits>

struct Spi::Context {
    int bus;
    int chip_select;
    SPI_HandleTypeDef* handle;
    GPIO_TypeDef* port;
    uint16_t pin;
};

Spi::Spi(int bus, int chip_select)
    : bus_(bus), chip_select_(chip_select) {
    // 同一总线上的两个独立片选：BMI088 加速度计、陀螺仪
    static Context contexts[] = {
        {.bus = 1, .chip_select = 1, .handle = &hspi1, .port = CS1_ACCEL_GPIO_Port, .pin = CS1_ACCEL_Pin},
        {.bus = 1, .chip_select = 2, .handle = &hspi1, .port = CS1_GYRO_GPIO_Port, .pin = CS1_GYRO_Pin},
    };
    for (auto& context : contexts) {
        if (context.bus == bus_ && context.chip_select == chip_select_) {
            context_ = &context;
            break;
        }
    }
}

bool Spi::transfer(const SpiTransfer& transfer) const {
    if (!context_ || !transfer.tx || !transfer.rx || transfer.size == 0 || transfer.size > std::numeric_limits<uint16_t>::max())
        return false;
    if (context_->handle->State != HAL_SPI_STATE_READY)
        return false;

    // 同步事务仅在普通上下文调用，整个传输期间保持中断可用
    HAL_GPIO_WritePin(context_->port, context_->pin, GPIO_PIN_RESET);
    const bool success = HAL_SPI_TransmitReceive(context_->handle, transfer.tx, transfer.rx, transfer.size, HAL_MAX_DELAY) == HAL_OK;
    HAL_GPIO_WritePin(context_->port, context_->pin, GPIO_PIN_SET);
    return success;
}
