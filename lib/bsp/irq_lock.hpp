#pragma once

#include <cstdint>

// 作用域关中断：构造时屏蔽可屏蔽中断，析构时恢复进入前的状态；可嵌套，中断内可用，临界区应尽量短
// 用法：{ const IrqLock lock; ... }
class IrqLock {
  public:
    IrqLock();
    ~IrqLock();

    // 复制会重复恢复中断状态
    IrqLock(const IrqLock&) = delete;
    IrqLock& operator=(const IrqLock&) = delete;

  private:
    uint32_t primask_; // 进入前的 PRIMASK
};
