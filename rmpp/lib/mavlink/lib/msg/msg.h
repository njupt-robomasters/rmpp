/** @file
 *  @brief MAVLink comm protocol generated from msg.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_MSG_H
#define MAVLINK_MSG_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_MSG.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_MSG_XML_HASH 3499625860539141686

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 178, 12, 12, 0, 0, 0}, {1, 20, 5, 5, 0, 0, 0}, {10, 133, 10, 10, 0, 0, 0}, {11, 84, 45, 45, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_MSG

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_imu.h"
#include "./mavlink_msg_referee.h"
#include "./mavlink_msg_aim.h"
#include "./mavlink_msg_ui.h"

// base include



#if MAVLINK_MSG_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_imu, MAVLINK_MESSAGE_INFO_referee, MAVLINK_MESSAGE_INFO_aim, MAVLINK_MESSAGE_INFO_ui}
# define MAVLINK_MESSAGE_NAMES {{ "aim", 10 }, { "imu", 0 }, { "referee", 1 }, { "ui", 11 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_MSG_H
