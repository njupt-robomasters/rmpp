#pragma once

#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "math/unit.hpp"

class Nav {
  public:
    struct Config {
        Cdc cdc;
        float timeout = 100 * ms;
    };

    struct {
        float x = 0;
        float y = 0;
    } target_position;

    struct {
        float vx = 0;
        float vy = 0;
    } chassis_speed;

  private:
    const float timeout;
    Timer chassis_speed_timer;

  public:
    explicit Nav(const Config& config);

    void update();
};
