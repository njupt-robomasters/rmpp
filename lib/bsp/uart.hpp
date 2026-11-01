#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

class Uart {
  public:
    // 接收数据仅在回调期间有效
    using RxCallback = std::function<void(const uint8_t*, std::size_t)>;

    // 编号不存在时各操作静默返回
    constexpr explicit Uart(const int id) : id_(id) {}

    // 成功时复制到通道发送缓冲区并启动 DMA；size 为 1–512，上次发送未完成时返回 false
    bool send(const uint8_t* data, std::size_t size) const;

    // 回调在接收中断内依序执行
    void add_rx_callback(RxCallback callback) const;

  private:
    int id_;
};
