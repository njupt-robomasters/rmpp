#include "bsp/uart.hpp"
#include "peripherals.hpp"
#include "usart.h"
#include <array>
#include <cstring>
#include <utility>
#include <vector>

struct Uart::State {
    int id;
    UART_HandleTypeDef* handle;
    std::vector<RxCallback> callbacks;
    std::array<std::uint8_t, 128> rx{};
    std::array<std::uint8_t, 256> tx{};

    struct Metrics {
        float last_callback_us = 0;
        float cpu_usage = 0;
        std::uint32_t error_count = 0;
    };

    volatile Metrics metrics{};
    CallbackTiming timing;

    void receive() {
        if (handle->hdmarx) {
            HAL_UARTEx_ReceiveToIdle_DMA(handle, rx.data(), rx.size());
            __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT);
        } else {
            HAL_UARTEx_ReceiveToIdle_IT(handle, rx.data(), rx.size());
        }
    }
};

Uart::Uart(int id) : id_(id) {
    static constinit State states[] = {
        {.id = 1, .handle = &huart1},
        {.id = 3, .handle = &huart3},
        {.id = 6, .handle = &huart6},
    };
    for (auto& state : states) {
        if (state.id == id_) {
            state_ = &state;
            break;
        }
    }
}

bool Uart::send(Bytes bytes) const {
    if (!state_ || !bytes.data || !bytes.size || bytes.size > state_->tx.size())
        return false;
    const auto primask = __get_PRIMASK();
    __disable_irq();
    bool success = false;
    if ((state_->handle->Init.Mode & UART_MODE_TX) && state_->handle->gState == HAL_UART_STATE_READY) {
        std::memcpy(state_->tx.data(), bytes.data, bytes.size);
        success = HAL_UART_Transmit_IT(state_->handle, state_->tx.data(), bytes.size) == HAL_OK;
    }
    __set_PRIMASK(primask);
    return success;
}

bool Uart::add_rx_callback(RxCallback callback) const {
    if (!state_ || !callback)
        return false;
    const auto primask = __get_PRIMASK();
    __disable_irq();
    state_->callbacks.push_back(std::move(callback));
    __set_PRIMASK(primask);
    return true;
}

void uart_init() {
    for (int id : {1, 3, 6}) {
        auto* state = Uart(id).state_;
        state->timing.window_start_ms = HAL_GetTick();
        state->receive();
    }
}

void uart_receive(int id, std::uint16_t size) {
    auto* state = Uart(id).state_;
    if (!state || HAL_UARTEx_GetRxEventType(state->handle) == HAL_UART_RXEVENT_HT)
        return;
    HAL_UART_AbortReceive(state->handle);
    if (size <= state->rx.size()) {
        const auto begin = DWT->CYCCNT;
        for (const auto& callback : state->callbacks)
            callback({state->rx.data(), size});
        const float duration_us = static_cast<std::uint32_t>(DWT->CYCCNT - begin) * (1000000.0f / SystemCoreClock);
        state->metrics.last_callback_us = duration_us;
        state->timing.record(duration_us, HAL_GetTick(), state->metrics.cpu_usage);
    }
    state->receive();
}

void uart_error(int id) {
    auto* state = Uart(id).state_;
    if (!state)
        return;
    state->metrics.error_count = state->metrics.error_count + 1;
    HAL_UART_AbortReceive(state->handle);
    state->receive();
}

static int uart_id(UART_HandleTypeDef* handle) {
    return handle == &huart1 ? 1 : handle == &huart3 ? 3
                               : handle == &huart6   ? 6
                                                     : 0;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* handle, uint16_t size) {
    uart_receive(uart_id(handle), size);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* handle) {
    uart_error(uart_id(handle));
}
