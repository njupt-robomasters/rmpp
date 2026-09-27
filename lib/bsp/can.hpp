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

    // 默认句柄不绑定通道，供电机型号默认配置常量初始化
    constexpr Can() = default;
    explicit Can(int id);

    explicit operator bool() const {
        return context_ != nullptr;
    }

    bool send(const CanFrame& frame) const;
    bool add_rx_callback(RxCallback callback) const;

    struct Context;
    Context* context_ = nullptr;
};
