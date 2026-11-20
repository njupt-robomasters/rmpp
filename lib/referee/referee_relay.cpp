#include "referee_relay.hpp"
#include "bsp/irq_lock.hpp"

RefereeRelay::RefereeRelay(const Config& config)
    : config(config) {
    // 串口：字节原样入队，待发上 CAN
    config.uart.add_rx_callback([this](const uint8_t* data, const size_t size) {
        to_can_drop += size - to_can.push(data, size);
    });

    // CAN：只收 rx_id 帧，载荷待发往串口
    config.can.add_rx_callback([this](const CanFrame& frame) {
        if (frame.id == this->config.rx_id && frame.size <= 8)
            to_uart_drop += frame.size - to_uart.push(frame.data, frame.size);
    });
}

void RefereeRelay::send() {
    // CAN：有空位时取至多 8 字节发 1 帧，不等凑满，避免帧尾滞留
    if (config.can.can_send()) {
        CanFrame frame{.id = config.tx_id};
        {
            // 关中断防止取出时接收中断写入
            const IrqLock lock;
            frame.size = static_cast<uint8_t>(to_can.pop(frame.data, sizeof(frame.data)));
        }
        if (frame.size != 0)
            config.can.send(frame);
    }

    // 串口：上次 DMA 完成后一次发出缓存的全部字节
    if (config.uart.can_send()) {
        uint8_t data[TO_UART_CAPACITY];
        size_t size = 0;
        {
            const IrqLock lock;
            size = to_uart.pop(data, sizeof(data));
        }
        if (size != 0)
            config.uart.send(data, size);
    }
}
