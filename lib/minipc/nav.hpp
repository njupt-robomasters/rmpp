#pragma once

#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "math/unit.hpp"

class Nav {
  public:
    struct Config {
        Cdc cdc;
        float timeout = 100 * ms;
    } config;

    struct {
        float x{};
        float y{};
    } target_position;

    struct {
        float vx{};
        float vy{};
    } chassis_speed;

    explicit Nav(const Config& config);
    void update();

  private:
    Timer timer_chassis_speed_;
};
