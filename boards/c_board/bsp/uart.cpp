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
#include "bsp/irq_lock.hpp"
#include "bsp/monitor.hpp"
#include "usart.h"
#include <cstring>
#include <vector>

static constexpr std::size_t BUFFER_CAPACITY = 512;

struct UartContext {
    int id;
    UART_HandleTypeDef* huart;
    uint8_t rx_buf[BUFFER_CAPACITY]{};
    uint8_t tx_buf[BUFFER_CAPACITY]{};
    std::vector<Uart::RxCallback> callbacks;
    Monitor monitor;
    volatile uint32_t error_count = 0; // UART 错误回调次数
};

static constinit UartContext contexts[] = {
    {.id = 1, .huart = &huart1},
    {.id = 3, .huart = &huart3},
    {.id = 6, .huart = &huart6},
};

static UartContext* find(const int id) {
    for (auto& context : contexts)
        if (context.id == id)
            return &context;
    return nullptr;
}

static UartContext* find(const UART_HandleTypeDef* huart) {
    for (auto& context : contexts)
        if (context.huart == huart)
            return &context;
    return nullptr;
}

// 启动一次空闲中断接收，屏蔽 DMA 半完成中断
static void start_receive(UartContext& context) {
    HAL_UARTEx_ReceiveToIdle_DMA(context.huart, context.rx_buf, BUFFER_CAPACITY);
    __HAL_DMA_DISABLE_IT(context.huart->hdmarx, DMA_IT_HT);
}

void uart_init() {
    for (auto& context : contexts)
        start_receive(context);
}

bool Uart::can_send() const {
    const auto* context = find(id);
    return context && context->huart->hdmatx && context->huart->gState == HAL_UART_STATE_READY;
}

void Uart::send(const uint8_t* data, const std::size_t size) const {
    if (!can_send())
        return;
    auto* context = find(id);

    std::memcpy(context->tx_buf, data, size);
    HAL_UART_Transmit_DMA(context->huart, context->tx_buf, size);
}

void Uart::add_rx_callback(RxCallback callback) const {
    auto* context = find(id);
    if (!context)
        return;

    const IrqLock lock;
    context->callbacks.push_back(std::move(callback));
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, const uint16_t size) {
    auto* context = find(huart);
    if (!context || HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
        return;

    context->monitor.begin();
    for (const auto& callback : context->callbacks)
        callback(context->rx_buf, size);
    context->monitor.end();

    start_receive(*context);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    auto* context = find(huart);
    if (!context)
        return;

    context->error_count = context->error_count + 1;
    start_receive(*context);
}
