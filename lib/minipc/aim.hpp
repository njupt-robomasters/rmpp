#pragma once

#include "bsp/cdc.hpp"
#include "bsp/timer.hpp"
#include "math/angle.hpp"
#include "math/unit.hpp"
#include "referee/referee.hpp"
#include <cmath>

class Aim {
  public:
    struct Config {
        Cdc cdc;
        float timeout = 100 * ms;
    };

    explicit Aim(const Config& config);
    void update();

  public:
    bool is_connect_vision = false;

    struct {
        Angle yaw;
        Angle pitch;
        Angle roll;
    } imu;

    struct {
        bool is_red = true;
        Referee::GameProgress game_progress = Referee::NOT_STARTED;
        float stage_remain_time = 0;
        float bullet_speed = 0;
    } referee;

    struct {
        bool is_detect = false;
        Angle yaw;
        Angle pitch;
        bool is_fire = false;
        float distance = 0;
    } vision;

    struct {
        float a0 = NAN;
    } insta360;

  private:
    const Config config_;
    Timer vision_timer_, insta360_timer_;
};
