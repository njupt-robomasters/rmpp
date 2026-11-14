#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

// USB CDC 虚拟串口；C 板只有 Cdc(1)，mc02 有 Cdc(1)、Cdc(2)
class Cdc {
  public:
    // 接收数据仅在回调期间有效
    using RxCallback = std::function<void(const uint8_t*, std::size_t)>;

  private:
    const int id;

  public:
    // 编号不存在时各操作静默返回
    constexpr explicit Cdc(const int id) : id(id) {}

    // 复制到端口发送缓冲区；size 为 1–2048，未枚举或上次发送未完成时丢弃
    void send(const uint8_t* data, std::size_t size) const;

    // 回调在 USB 中断内依序执行
    void add_rx_callback(RxCallback callback) const;
};
