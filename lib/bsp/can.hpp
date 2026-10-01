#pragma once

#include <cstdint>
#include <functional>

struct CanFrame {
    uint32_t id = 0;
    bool is_extend = false;
    bool is_remote = false;
    uint8_t size = 0;
    uint8_t data[8]{};
};

class Can {
  public:
    using RxCallback = std::function<void(const CanFrame&)>;

    explicit Can(int id);

    // 返回发送请求是否进入硬件邮箱
    bool send(const CanFrame& frame) const;
    void add_rx_callback(RxCallback callback) const;

    struct Context;
    Context* context_ = nullptr;
};
