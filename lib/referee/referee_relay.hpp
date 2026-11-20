#pragma once

#include "bsp/can.hpp"
#include "bsp/uart.hpp"
#include "container/fifo.hpp"

// 裁判系统串口与 CAN 原样转发字节流，供经 CAN 接收的 Referee 使用
// 串口收到的字节按序切成至多 8 字节的 tx_id 帧发出；收到的 rx_id 帧载荷按序拼接后由串口发出
class RefereeRelay {
  private:
    static constexpr size_t TO_CAN_CAPACITY = 1024; // 串口 → CAN，B；1 Hz 定频报文与触发报文叠加的最坏突发约 500 B，发出 2 kB/s
    static constexpr size_t TO_UART_CAPACITY = 256; // CAN → 串口，B；两次 send() 间至多积压 32 B

  public:
    struct Config {
        Uart uart;             // 接电源管理模块，115200
        Can can;               // 接 Referee 所在板
        uint32_t tx_id = 0x00; // 本板发送 CAN ID，载荷来自串口；对应 Referee rx_id
        uint32_t rx_id = 0x01; // 本板接收 CAN ID，载荷发往串口；对应 Referee tx_id
    };

  private:
    const Config config;

    // 已满丢弃的字节数，接收中断累加；SWD 观察
    uint32_t to_can_drop = 0, to_uart_drop = 0;

    // 接收中断写入，主循环关中断取出
    Fifo<uint8_t, TO_CAN_CAPACITY> to_can;   // 待发上 CAN
    Fifo<uint8_t, TO_UART_CAPACITY> to_uart; // 待发往串口

  public:
    // 实例在 BSP 回调有效期间不得销毁或移动
    explicit RefereeRelay(const Config& config);

    RefereeRelay(const RefereeRelay&) = delete;

    // 仅在主循环调用：CAN 有空位时发 1 帧 tx_id，串口空闲时一次发出 to_uart 全部字节
    void send();
};
