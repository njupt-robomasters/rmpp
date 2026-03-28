#pragma once

#include "misc/LED.hpp"
#include "misc/Buzzer.hpp"
#include "flashdb/FlashDB.hpp"
#include "rc/RC.hpp"
#include "mavlink/Mavlink.hpp"
#include "referee/Referee.hpp"
#include "ui/UI.hpp"

inline LED led({});
inline Buzzer buzzer;
inline FlashDB flashdb("hero");
inline RC rc({}, {});
inline Mavlink mavlink({});
inline Referee referee({});
inline UI ui({.max_bullet = 200 * default_unit});