#pragma once

#include <cstdint>

struct SpiTransfer {
    const uint8_t* tx = nullptr;
    uint8_t* rx = nullptr;
    std::size_t size = 0;
};

class Spi {
  public:
    Spi() = default;
    Spi(int bus, int chip_select);

    explicit operator bool() const {
        return context_ != nullptr;
    }

    bool transfer(const SpiTransfer& transfer) const;

  private:
    struct Context;
    int bus_ = 0, chip_select_ = 0;
    Context* context_ = nullptr;
};
