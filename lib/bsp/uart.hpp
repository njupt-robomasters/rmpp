#pragma once

#include <cstdint>
#include <functional>

class Uart {
  public:
    // 接收数据仅在回调期间有效
    using RxCallback = std::function<void(const uint8_t*, std::size_t)>;

    explicit Uart(int id);

    explicit operator bool() const {
        return context_ != nullptr;
    }

    // 成功时复制到通道发送缓冲区，DMA 发送期间保持有效
    bool send(const uint8_t* data, std::size_t size) const;
    bool add_rx_callback(RxCallback callback) const;

    struct Context;
    Context* context_ = nullptr;
};
