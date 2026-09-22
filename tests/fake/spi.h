#pragma once

#include "stm32f4xx.h"
#include <cassert>
#include <utility>
#include <vector>

struct GPIO_TypeDef {};

inline GPIO_TypeDef gpio;
inline auto* CS1_ACCEL_GPIO_Port = &gpio;
inline auto* CS1_GYRO_GPIO_Port = &gpio;
inline constexpr std::uint16_t CS1_ACCEL_Pin = 1, CS1_GYRO_Pin = 2;
inline constexpr int GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1;
inline constexpr int HAL_SPI_STATE_READY = 1, HAL_SPI_STATE_BUSY = 2;

struct SPI_HandleTypeDef {
    int State = HAL_SPI_STATE_READY;
};

inline SPI_HandleTypeDef hspi1;

struct SpiPacket {
    std::uint16_t pin;
    std::uint32_t timeout;
    std::vector<std::uint8_t> tx;
};

inline std::vector<SpiPacket> spi_packets;
inline std::vector<std::pair<std::uint16_t, int>> spi_edges;
inline std::uint16_t selected_pin = 0;
inline HAL_StatusTypeDef spi_result = HAL_OK;

inline void HAL_GPIO_WritePin(GPIO_TypeDef*, std::uint16_t pin, int value) {
    spi_edges.emplace_back(pin, value);
    selected_pin = value == GPIO_PIN_RESET ? pin : 0;
}

inline HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*, const std::uint8_t* tx, std::uint8_t* rx, std::uint16_t size, std::uint32_t timeout) {
    assert(selected_pin != 0 && irq_mask == 0);
    spi_packets.push_back({.pin = selected_pin, .timeout = timeout, .tx = {tx, tx + size}});
    for (unsigned i = 0; i < size; ++i)
        rx[i] = i;
    return spi_result;
}
