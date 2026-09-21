#include "board/uart.hpp"
#include "board/timer.hpp"
#include "irq_lock.hpp"
#include "peripherals.hpp"
#include "usart.h"
#include <array>
#include <cstring>
#include <utility>
#include <vector>

namespace rmpp {
struct Uart::State {
    UART_HandleTypeDef* handle;
    std::vector<RxCallback> callbacks;
    std::array<std::uint8_t, 128> rx{};
    std::array<std::uint8_t, 256> tx{};
    Timer timer{0};
    float cpu_usage = 0;
    std::uint32_t error_count = 0;

    void receive() {
        if (handle->hdmarx) {
            HAL_UARTEx_ReceiveToIdle_DMA(handle, rx.data(), rx.size());
            __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT);
        } else {
            HAL_UARTEx_ReceiveToIdle_IT(handle, rx.data(), rx.size());
        }
    }
};

Uart::State* Uart::find(int id) {
    switch (id) {
    case 1: {
        static constinit State state{&huart1};
        return &state;
    }
    case 3: {
        static constinit State state{&huart3};
        return &state;
    }
    case 6: {
        static constinit State state{&huart6};
        return &state;
    }
    default:
        return nullptr;
    }
}

bool Uart::send(Bytes bytes) const {
    if (!state_ || !bytes.data || !bytes.size || bytes.size > state_->tx.size())
        return false;
    IrqLock lock;
    if (!(state_->handle->Init.Mode & UART_MODE_TX) || state_->handle->gState != HAL_UART_STATE_READY)
        return false;
    std::memcpy(state_->tx.data(), bytes.data, bytes.size);
    return HAL_UART_Transmit_IT(state_->handle, state_->tx.data(), bytes.size) == HAL_OK;
}

bool Uart::add_rx_callback(RxCallback callback) const {
    if (!state_ || !callback)
        return false;
    IrqLock lock;
    state_->callbacks.push_back(std::move(callback));
    return true;
}

float Uart::cpu_usage() const { return state_ ? state_->cpu_usage : 0; }
std::uint32_t Uart::error_count() const { return state_ ? state_->error_count : 0; }

void uart_init() {
    for (int id : {1, 3, 6})
        Uart::find(id)->receive();
}

void uart_receive(int id, std::uint16_t size) {
    auto* state = Uart::find(id);
    if (!state || HAL_UARTEx_GetRxEventType(state->handle) == HAL_UART_RXEVENT_HT)
        return;
    HAL_UART_AbortReceive(state->handle);
    const float interval = state->timer.update_dt();
    if (size <= state->rx.size()) {
        for (const auto& callback : state->callbacks)
            callback({state->rx.data(), size});
    }
    state->cpu_usage = interval > 0 ? state->timer.get_dt() / interval : 0;
    state->receive();
}

void uart_error(int id) {
    auto* state = Uart::find(id);
    if (!state)
        return;
    ++state->error_count;
    HAL_UART_AbortReceive(state->handle);
    state->receive();
}
} // namespace rmpp

static int uart_id(UART_HandleTypeDef* handle) {
    return handle == &huart1 ? 1 : handle == &huart3 ? 3
                               : handle == &huart6   ? 6
                                                     : 0;
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* handle, uint16_t size) {
    rmpp::uart_receive(uart_id(handle), size);
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* handle) {
    rmpp::uart_error(uart_id(handle));
}
