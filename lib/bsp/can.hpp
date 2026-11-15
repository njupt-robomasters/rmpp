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

  private:
    const int id;

  public:
    // 编号不存在时各操作静默返回
    constexpr explicit Can(const int id) : id(id) {}

    // 硬件邮箱或发送队列有空位
    bool can_send() const;

    // 写入硬件邮箱或发送队列，无空位时丢弃
    void send(const CanFrame& frame) const;

    // 回调在接收中断内依序执行
    void add_rx_callback(RxCallback callback) const;
};
