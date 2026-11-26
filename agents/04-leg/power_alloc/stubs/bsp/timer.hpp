#pragma once

#include <cstdint>

class Timer {
  private:
    float dt = 0;
    uint64_t last_cycles = 0;

  public:
    static void delay(float seconds);

    float elapsed() const;

    float restart();
};
