#pragma once

#include <cstdint>
#include <functional>

class Uart {
  public:
    // 接收数据仅在回调期间有效
    using RxCallback = std::function<void(const uint8_t*, size_t)>;

    explicit Uart(int id);

    // 成功时复制到通道发送缓冲区并启动 DMA；size 为 1–512
    bool send(const uint8_t* data, size_t size) const;
    void add_rx_callback(RxCallback callback) const;

    struct Context;
    Context* context_ = nullptr;
};
