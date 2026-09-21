#pragma once

#include "stm32f4xx.h"

namespace rmpp {
// Restore the caller's interrupt mask, including inside another lock.
class IrqLock {
  public:
    IrqLock() : primask_(__get_PRIMASK()) { __disable_irq(); }
    ~IrqLock() { __set_PRIMASK(primask_); }
    IrqLock(const IrqLock&) = delete;
    IrqLock& operator=(const IrqLock&) = delete;

  private:
    uint32_t primask_;
};
} // namespace rmpp
