#include "bsp/spi.hpp"
#include "spi.h"
#include <limits>

struct Spi::State {
    int bus;
    int chip_select;
    SPI_HandleTypeDef* handle;
    GPIO_TypeDef* port;
    std::uint16_t pin;
};

Spi::Spi(int bus, int chip_select) : bus_(bus), chip_select_(chip_select) {
    // 同一总线上的两个独立片选：BMI088 加速度计、陀螺仪。
    static State states[] = {
        {.bus = 1, .chip_select = 1, .handle = &hspi1, .port = CS1_ACCEL_GPIO_Port, .pin = CS1_ACCEL_Pin},
        {.bus = 1, .chip_select = 2, .handle = &hspi1, .port = CS1_GYRO_GPIO_Port, .pin = CS1_GYRO_Pin},
    };
    for (auto& state : states) {
        if (state.bus == bus_ && state.chip_select == chip_select_) {
            state_ = &state;
            break;
        }
    }
}

bool Spi::transfer(const SpiTransfer& transfer) const {
    if (!state_ || !transfer.tx || !transfer.rx || transfer.size == 0 || transfer.size > std::numeric_limits<std::uint16_t>::max())
        return false;
    if (state_->handle->State != HAL_SPI_STATE_READY)
        return false;

    // 同步事务仅在普通上下文调用，整个传输期间保持中断可用。
    HAL_GPIO_WritePin(state_->port, state_->pin, GPIO_PIN_RESET);
    const bool success = HAL_SPI_TransmitReceive(state_->handle, transfer.tx, transfer.rx, transfer.size, 10) == HAL_OK;
    HAL_GPIO_WritePin(state_->port, state_->pin, GPIO_PIN_SET);
    return success;
}
