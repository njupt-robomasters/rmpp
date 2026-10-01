/*
 * USART1：Connectivity → USART1
 *
 * | 页面               | 参数                           | 设置                        |
 * | ------------------ | ------------------------------ | --------------------------- |
 * | Parameter Settings | Mode                           | Asynchronous                |
 * | Parameter Settings | Baud Rate                      | 115200                      |
 * | Parameter Settings | Word Length (including Parity) | 8 Bits                      |
 * | Parameter Settings | Parity                         | None                        |
 * | Parameter Settings | Stop Bits                      | 1                           |
 * | Parameter Settings | Data Direction                 | Receive and Transmit        |
 * | DMA Settings       | Add                            | USART1_RX、USART1_TX        |
 * | NVIC Settings      | USART1 global interrupt        | 勾选，Preemption Priority=0 |
 *
 * USART3：Connectivity → USART3
 *
 * | 页面               | 参数                           | 设置                        |
 * | ------------------ | ------------------------------ | --------------------------- |
 * | Parameter Settings | Mode                           | Asynchronous                |
 * | Parameter Settings | Baud Rate                      | 100000                      |
 * | Parameter Settings | Word Length (including Parity) | 9 Bits                      |
 * | Parameter Settings | Parity                         | Even                        |
 * | Parameter Settings | Stop Bits                      | 2                           |
 * | Parameter Settings | Data Direction                 | Receive Only                |
 * | DMA Settings       | Add                            | USART3_RX                   |
 * | NVIC Settings      | USART3 global interrupt        | 勾选，Preemption Priority=5 |
 *
 * USART6：Connectivity → USART6
 *
 * | 页面               | 参数                           | 设置                        |
 * | ------------------ | ------------------------------ | --------------------------- |
 * | Parameter Settings | Mode                           | Asynchronous                |
 * | Parameter Settings | Baud Rate                      | 921600                      |
 * | Parameter Settings | Word Length (including Parity) | 8 Bits                      |
 * | Parameter Settings | Parity                         | None                        |
 * | Parameter Settings | Stop Bits                      | 1                           |
 * | Parameter Settings | Data Direction                 | Receive and Transmit        |
 * | DMA Settings       | Add                            | USART6_RX、USART6_TX        |
 * | NVIC Settings      | USART6 global interrupt        | 勾选，Preemption Priority=0 |
 *
 * | 通道   | RX   | TX   |
 * | ------ | ---- | ---- |
 * | USART1 | PB7  | PA9  |
 * | USART3 | PC11 | -    |
 * | USART6 | PG9  | PG14 |
 */

#include "bsp/uart.hpp"
#include "bsp/monitor.hpp"
#include "usart.h"
#include <cstring>

static constexpr std::size_t UART_BUFFER_CAPACITY = 512;

struct Uart::Context {
    int id;
    UART_HandleTypeDef* huart;
    uint8_t rx_buf[UART_BUFFER_CAPACITY]{};
    uint8_t tx_buf[UART_BUFFER_CAPACITY]{};
    std::vector<RxCallback> callbacks;
    Monitor monitor;
    volatile uint32_t error_count = 0; // UART 错误回调次数
};

void uart_init() {
    for (const int id : {1, 3, 6}) {
        auto* context = Uart(id).context_;
        HAL_UARTEx_ReceiveToIdle_DMA(context->huart, context->rx_buf, UART_BUFFER_CAPACITY);
        __HAL_DMA_DISABLE_IT(context->huart->hdmarx, DMA_IT_HT);
    }
}

Uart::Uart(const int id) {
    static constinit Context contexts[] = {
        {.id = 1, .huart = &huart1},
        {.id = 3, .huart = &huart3},
        {.id = 6, .huart = &huart6},
    };
    if (id == 1)
        context_ = &contexts[0];
    else if (id == 3)
        context_ = &contexts[1];
    else if (id == 6)
        context_ = &contexts[2];
}

bool Uart::send(const uint8_t* data, const size_t size) const {
    if (!context_ || !context_->huart->hdmatx || context_->huart->gState != HAL_UART_STATE_READY)
        return false;

    std::memcpy(context_->tx_buf, data, size);
    return HAL_UART_Transmit_DMA(context_->huart, context_->tx_buf, size) == HAL_OK;
}

void Uart::add_rx_callback(RxCallback callback) const {
    if (!context_)
        return;

    const auto primask = __get_PRIMASK();
    __disable_irq();
    context_->callbacks.push_back(std::move(callback));
    __set_PRIMASK(primask);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    Uart::Context* context = nullptr;
    if (huart == &huart1)
        context = Uart(1).context_;
    else if (huart == &huart3)
        context = Uart(3).context_;
    else if (huart == &huart6)
        context = Uart(6).context_;

    // 忽略半完成中断
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
        return;

    context->monitor.begin();
    for (const auto& callback : context->callbacks)
        callback(context->rx_buf, size);
    context->monitor.end();

    // 再次启动串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, context->rx_buf, UART_BUFFER_CAPACITY);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 屏蔽半完成中断
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* handle) {
    Uart::Context* context = nullptr;
    if (handle == &huart1)
        context = Uart(1).context_;
    else if (handle == &huart3)
        context = Uart(3).context_;
    else if (handle == &huart6)
        context = Uart(6).context_;

    context->error_count += 1;

    // 重启串口接收
    HAL_UARTEx_ReceiveToIdle_DMA(handle, context->rx_buf, UART_BUFFER_CAPACITY);
    __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT); // 屏蔽半完成中断
}
