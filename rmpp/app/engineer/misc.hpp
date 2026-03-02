#pragma once

#include "module/LED.hpp"
#include "buzzer/Buzzer.hpp"
#include "rc/RC.hpp"
#include "mavlink/Mavlink.hpp"
#include "referee/Referee.hpp"
#include "ui/UI.hpp"
#include "flashdb/FlashDB.hpp"

inline LED led({});
inline Buzzer buzzer;
inline RC rc({}, {});
inline Mavlink mavlink;
inline Referee referee({});
inline UI ui({});
inline FlashDB flashdb("engineer");