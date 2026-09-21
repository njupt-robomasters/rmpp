#pragma once

#include "flashdb/flash_db.hpp"
#include "mavlink/mavlink.hpp"
#include "misc/buzzer.hpp"
#include "misc/led.hpp"
#include "rc/rc.hpp"
#include "referee/referee.hpp"
#include "ui/ui.hpp"

inline Led led({});
inline Buzzer buzzer(rmpp::board.pwm(4, 3));
inline FlashDb flashdb("engineer");
inline Rc rc({}, {});
inline Mavlink mavlink;
inline Referee referee({});
inline Ui ui({});