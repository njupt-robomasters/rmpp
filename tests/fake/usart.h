#pragma once
#include "stm32f4xx.h"

inline constexpr unsigned UART_MODE_TX = 1, UART_MODE_RX = 2, UART_MODE_TX_RX = 3;
inline constexpr unsigned HAL_UART_STATE_READY = 1, HAL_UART_STATE_BUSY_TX = 2;
inline constexpr unsigned HAL_UART_RXEVENT_HT = 0, HAL_UART_RXEVENT_TC = 1, HAL_UART_RXEVENT_IDLE = 2, DMA_IT_HT = 1;

struct DMA_HandleTypeDef {
    unsigned disabled = 0;
};

struct UART_HandleTypeDef {
    struct {
        unsigned Mode = UART_MODE_TX_RX;
    } Init;

    unsigned gState = HAL_UART_STATE_READY;
    DMA_HandleTypeDef* hdmarx = nullptr;
    std::uint8_t* rx = nullptr;
    const std::uint8_t* tx = nullptr;
    unsigned rx_size = 0, tx_size = 0, rearms = 0, event = HAL_UART_RXEVENT_IDLE;
};

inline UART_HandleTypeDef huart1, huart3, huart6;

inline HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef* h, std::uint8_t* data, unsigned size) {
    h->rx = data;
    h->rx_size = size;
    ++h->rearms;
    return HAL_OK;
}

inline HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef* h, std::uint8_t* data, unsigned size) {
    return HAL_UARTEx_ReceiveToIdle_IT(h, data, size);
}

inline void __HAL_DMA_DISABLE_IT(DMA_HandleTypeDef* h, unsigned mask) {
    h->disabled |= mask;
}

inline unsigned HAL_UARTEx_GetRxEventType(UART_HandleTypeDef* h) {
    return h->event;
}

inline HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*) {
    return HAL_OK;
}

inline HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef* h, const std::uint8_t* data, unsigned size) {
    if (h->gState != HAL_UART_STATE_READY)
        return HAL_BUSY;
    h->gState = HAL_UART_STATE_BUSY_TX;
    h->tx = data;
    h->tx_size = size;
    return HAL_OK;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*, std::uint16_t);
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef*);
