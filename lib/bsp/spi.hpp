#pragma once

#include <cstddef>
#include <cstdint>

class Spi {
  public:
    Spi(int bus, int chip_select);

    void transfer(const uint8_t* tx_buf, uint8_t* rx_buf, std::size_t size) const;

  private:
    struct Context;
    Context* context_ = nullptr;
};
