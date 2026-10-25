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
    } config;

    bool is_connect_vision = false;

    struct {
        Angle yaw;
        Angle pitch;
        Angle roll;
    } imu;

    struct {
        bool is_red = true;
        Referee::GameProgress game_progress = Referee::NOT_STARTED;
        float stage_remain_time{};
        float bullet_speed{};
    } referee;

    struct {
        bool is_detect = false;
        Angle yaw;
        Angle pitch;
        bool is_fire = false;
        float distance{};
    } vision;

    struct {
        float a0 = NAN;
    } insta360;

    explicit Aim(const Config& config);
    void update();

  private:
    Timer timer_vision_, timer_insta360_;
};
