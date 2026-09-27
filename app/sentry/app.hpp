#pragma once

#include "chassis.hpp"
#include "gimbal.hpp"
#include "math/unit.hpp"
#include "misc.hpp"
#include "shooter.hpp"

inline bool sentry_in_game() {
    return referee.game.game_progress == Referee::GAMING && referee.robot.type == Referee::SENTRY;
}

inline bool sentry_active() {
    return sentry_in_game() || rc.is_fn;
}
