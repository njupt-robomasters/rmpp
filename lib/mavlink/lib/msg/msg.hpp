/** @file
 *	@brief MAVLink comm protocol generated from msg.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace msg {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (through @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 4> MESSAGE_ENTRIES {{ {0, 178, 12, 12, 0, 0, 0}, {1, 20, 5, 5, 0, 0, 0}, {10, 133, 10, 10, 0, 0, 0}, {11, 84, 45, 45, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 2;


// ENUM DEFINITIONS




} // namespace msg
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_imu.hpp"
#include "./mavlink_msg_referee.hpp"
#include "./mavlink_msg_aim.hpp"
#include "./mavlink_msg_ui.hpp"

// base include

