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

    // 编号不存在时各操作静默返回
    constexpr explicit Can(const int id) : id_(id) {}

    // 返回发送请求是否进入硬件邮箱或发送队列
    bool send(const CanFrame& frame) const;

    // 回调在接收中断内依序执行
    void add_rx_callback(RxCallback callback) const;

  private:
    int id_;
};
