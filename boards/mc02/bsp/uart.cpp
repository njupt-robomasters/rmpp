/*
 * 串口：全部 DMA + 空闲中断接收，DMA 发送；参数由 CubeMX 生成
 * | 通道    | 波特率 | 格式              | RX   | TX   | 用途          |
 * | ------- | ------ | ----------------- | ---- | ---- | ------------- |
 * | USART1  | 921600 | 8N1               | PA10 | PA9  |               |
 * | UART5   | 100000 | 9 位偶校验 2 停止 | PD2  | -    | DBUS，只接收 |
 * | UART7   | 115200 | 8N1               | PE7  | PE8  |               |
 * | USART10 | 115200 | 8N1               | PE2  | PE3  |               |
 *
 * DMA1 不能访问 DTCM，收发缓冲放在 .dma_buffer（RAM_D1，见 dma_buffer.ld）
 * D-Cache 已开启且未用 MPU：发送前清理、接收通知前失效对应缓存行；缓冲按 32 字节对齐，容量为 32 的倍数
 */

#include "bsp/uart.hpp"
#include "bsp/monitor.hpp"
#include "usart.h"
#include <cstring>
#include <vector>

static constexpr std::size_t BUFFER_CAPACITY = 512;

struct DmaBuffer {
    alignas(32) uint8_t rx[BUFFER_CAPACITY];
    alignas(32) uint8_t tx[BUFFER_CAPACITY];
};

__attribute__((section(".dma_buffer"))) static DmaBuffer buffers[4];

struct UartContext {
    int id;
    UART_HandleTypeDef* huart;
    DmaBuffer* buffer;
    std::vector<Uart::RxCallback> callbacks;
    Monitor monitor;
    volatile uint32_t error_count = 0; // UART 错误回调次数
};

static constinit UartContext contexts[] = {
    {.id = 1, .huart = &huart1, .buffer = &buffers[0]},
    {.id = 5, .huart = &huart5, .buffer = &buffers[1]},
    {.id = 7, .huart = &huart7, .buffer = &buffers[2]},
    {.id = 10, .huart = &huart10, .buffer = &buffers[3]},
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
    HAL_UARTEx_ReceiveToIdle_DMA(context.huart, context.buffer->rx, BUFFER_CAPACITY);
    __HAL_DMA_DISABLE_IT(context.huart->hdmarx, DMA_IT_HT);
}

void uart_init() {
    for (auto& context : contexts)
        start_receive(context);
}

bool Uart::send(const uint8_t* data, const std::size_t size) const {
    auto* context = find(id_);
    if (!context)
        return false;
    if (!context->huart->hdmatx || context->huart->gState != HAL_UART_STATE_READY)
        return false;

    // 写回数据后再启动 DMA 读取
    std::memcpy(context->buffer->tx, data, size);
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(context->buffer->tx), static_cast<int32_t>(size));
    return HAL_UART_Transmit_DMA(context->huart, context->buffer->tx, size) == HAL_OK;
}

void Uart::add_rx_callback(RxCallback callback) const {
    auto* context = find(id_);
    if (!context)
        return;

    const auto primask = __get_PRIMASK();
    __disable_irq();
    context->callbacks.push_back(std::move(callback));
    __set_PRIMASK(primask);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, const uint16_t size) {
    auto* context = find(huart);
    if (!context || HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
        return;

    // 丢弃旧缓存行，读取 DMA 写入的数据
    SCB_InvalidateDCache_by_Addr(context->buffer->rx, BUFFER_CAPACITY);

    context->monitor.begin();
    for (const auto& callback : context->callbacks)
        callback(context->buffer->rx, size);
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
