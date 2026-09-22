#pragma once

#include <cstddef>
#include <cstdint>

struct SpiTransfer {
    const std::uint8_t* tx = nullptr;
    std::uint8_t* rx = nullptr;
    std::size_t size = 0;
};

class Spi {
  public:
    Spi() = default;
    Spi(int bus, int chip_select);

    explicit operator bool() const {
        return state_ != nullptr;
    }

    bool transfer(const SpiTransfer& transfer) const;

  private:
    struct State;
    int bus_ = 0, chip_select_ = 0;
    State* state_ = nullptr;
};
