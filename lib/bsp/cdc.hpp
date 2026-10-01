#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

class Cdc {
  public:
    // 接收数据仅在回调期间有效
    using RxCallback = std::function<void(const uint8_t*, std::size_t)>;

    explicit Cdc(int id);

    // 成功时复制到 STM32 CDC 发送缓冲区；size 不超过 APP_TX_DATA_SIZE
    bool send(const uint8_t* data, std::size_t size) const;
    void add_rx_callback(RxCallback callback) const;

    struct Context;
    Context* context_ = nullptr;
};
