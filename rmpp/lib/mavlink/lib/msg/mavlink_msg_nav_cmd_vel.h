#pragma once
// MESSAGE nav_cmd_vel PACKING

#define MAVLINK_MSG_ID_nav_cmd_vel 20


typedef struct __mavlink_nav_cmd_vel_t {
 float vel_x; /*< [m_s] x*/
 float vel_y; /*< [m_s] y*/
} mavlink_nav_cmd_vel_t;

#define MAVLINK_MSG_ID_nav_cmd_vel_LEN 8
#define MAVLINK_MSG_ID_nav_cmd_vel_MIN_LEN 8
#define MAVLINK_MSG_ID_20_LEN 8
#define MAVLINK_MSG_ID_20_MIN_LEN 8

#define MAVLINK_MSG_ID_nav_cmd_vel_CRC 58
#define MAVLINK_MSG_ID_20_CRC 58



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_nav_cmd_vel { \
    20, \
    "nav_cmd_vel", \
    2, \
    {  { "vel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_nav_cmd_vel_t, vel_x) }, \
         { "vel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_nav_cmd_vel_t, vel_y) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_nav_cmd_vel { \
    "nav_cmd_vel", \
    2, \
    {  { "vel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_nav_cmd_vel_t, vel_x) }, \
         { "vel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_nav_cmd_vel_t, vel_y) }, \
         } \
}
#endif

/**
 * @brief Pack a nav_cmd_vel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vel_x [m_s] x
 * @param vel_y [m_s] y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nav_cmd_vel_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float vel_x, float vel_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_nav_cmd_vel_LEN];
    _mav_put_float(buf, 0, vel_x);
    _mav_put_float(buf, 4, vel_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_nav_cmd_vel_LEN);
#else
    mavlink_nav_cmd_vel_t packet;
    packet.vel_x = vel_x;
    packet.vel_y = vel_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_nav_cmd_vel_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_nav_cmd_vel;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_nav_cmd_vel_MIN_LEN, MAVLINK_MSG_ID_nav_cmd_vel_LEN, MAVLINK_MSG_ID_nav_cmd_vel_CRC);
}

/**
 * @brief Pack a nav_cmd_vel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param vel_x [m_s] x
 * @param vel_y [m_s] y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nav_cmd_vel_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float vel_x, float vel_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_nav_cmd_vel_LEN];
    _mav_put_float(buf, 0, vel_x);
    _mav_put_float(buf, 4, vel_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_nav_cmd_vel_LEN);
#else
    mavlink_nav_cmd_vel_t packet;
    packet.vel_x = vel_x;
    packet.vel_y = vel_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_nav_cmd_vel_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_nav_cmd_vel;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_nav_cmd_vel_MIN_LEN, MAVLINK_MSG_ID_nav_cmd_vel_LEN, MAVLINK_MSG_ID_nav_cmd_vel_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_nav_cmd_vel_MIN_LEN, MAVLINK_MSG_ID_nav_cmd_vel_LEN);
#endif
}

/**
 * @brief Pack a nav_cmd_vel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vel_x [m_s] x
 * @param vel_y [m_s] y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nav_cmd_vel_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float vel_x,float vel_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_nav_cmd_vel_LEN];
    _mav_put_float(buf, 0, vel_x);
    _mav_put_float(buf, 4, vel_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_nav_cmd_vel_LEN);
#else
    mavlink_nav_cmd_vel_t packet;
    packet.vel_x = vel_x;
    packet.vel_y = vel_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_nav_cmd_vel_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_nav_cmd_vel;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_nav_cmd_vel_MIN_LEN, MAVLINK_MSG_ID_nav_cmd_vel_LEN, MAVLINK_MSG_ID_nav_cmd_vel_CRC);
}

/**
 * @brief Encode a nav_cmd_vel struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param nav_cmd_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nav_cmd_vel_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nav_cmd_vel_t* nav_cmd_vel)
{
    return mavlink_msg_nav_cmd_vel_pack(system_id, component_id, msg, nav_cmd_vel->vel_x, nav_cmd_vel->vel_y);
}

/**
 * @brief Encode a nav_cmd_vel struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nav_cmd_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nav_cmd_vel_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_nav_cmd_vel_t* nav_cmd_vel)
{
    return mavlink_msg_nav_cmd_vel_pack_chan(system_id, component_id, chan, msg, nav_cmd_vel->vel_x, nav_cmd_vel->vel_y);
}

/**
 * @brief Encode a nav_cmd_vel struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param nav_cmd_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nav_cmd_vel_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_nav_cmd_vel_t* nav_cmd_vel)
{
    return mavlink_msg_nav_cmd_vel_pack_status(system_id, component_id, _status, msg,  nav_cmd_vel->vel_x, nav_cmd_vel->vel_y);
}

/**
 * @brief Send a nav_cmd_vel message
 * @param chan MAVLink channel to send the message
 *
 * @param vel_x [m_s] x
 * @param vel_y [m_s] y
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nav_cmd_vel_send(mavlink_channel_t chan, float vel_x, float vel_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_nav_cmd_vel_LEN];
    _mav_put_float(buf, 0, vel_x);
    _mav_put_float(buf, 4, vel_y);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_nav_cmd_vel, buf, MAVLINK_MSG_ID_nav_cmd_vel_MIN_LEN, MAVLINK_MSG_ID_nav_cmd_vel_LEN, MAVLINK_MSG_ID_nav_cmd_vel_CRC);
#else
    mavlink_nav_cmd_vel_t packet;
    packet.vel_x = vel_x;
    packet.vel_y = vel_y;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_nav_cmd_vel, (const char *)&packet, MAVLINK_MSG_ID_nav_cmd_vel_MIN_LEN, MAVLINK_MSG_ID_nav_cmd_vel_LEN, MAVLINK_MSG_ID_nav_cmd_vel_CRC);
#endif
}

/**
 * @brief Send a nav_cmd_vel message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_nav_cmd_vel_send_struct(mavlink_channel_t chan, const mavlink_nav_cmd_vel_t* nav_cmd_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_nav_cmd_vel_send(chan, nav_cmd_vel->vel_x, nav_cmd_vel->vel_y);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_nav_cmd_vel, (const char *)nav_cmd_vel, MAVLINK_MSG_ID_nav_cmd_vel_MIN_LEN, MAVLINK_MSG_ID_nav_cmd_vel_LEN, MAVLINK_MSG_ID_nav_cmd_vel_CRC);
#endif
}

#if MAVLINK_MSG_ID_nav_cmd_vel_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_nav_cmd_vel_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vel_x, float vel_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vel_x);
    _mav_put_float(buf, 4, vel_y);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_nav_cmd_vel, buf, MAVLINK_MSG_ID_nav_cmd_vel_MIN_LEN, MAVLINK_MSG_ID_nav_cmd_vel_LEN, MAVLINK_MSG_ID_nav_cmd_vel_CRC);
#else
    mavlink_nav_cmd_vel_t *packet = (mavlink_nav_cmd_vel_t *)msgbuf;
    packet->vel_x = vel_x;
    packet->vel_y = vel_y;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_nav_cmd_vel, (const char *)packet, MAVLINK_MSG_ID_nav_cmd_vel_MIN_LEN, MAVLINK_MSG_ID_nav_cmd_vel_LEN, MAVLINK_MSG_ID_nav_cmd_vel_CRC);
#endif
}
#endif

#endif

// MESSAGE nav_cmd_vel UNPACKING


/**
 * @brief Get field vel_x from nav_cmd_vel message
 *
 * @return [m_s] x
 */
static inline float mavlink_msg_nav_cmd_vel_get_vel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vel_y from nav_cmd_vel message
 *
 * @return [m_s] y
 */
static inline float mavlink_msg_nav_cmd_vel_get_vel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a nav_cmd_vel message into a struct
 *
 * @param msg The message to decode
 * @param nav_cmd_vel C-struct to decode the message contents into
 */
static inline void mavlink_msg_nav_cmd_vel_decode(const mavlink_message_t* msg, mavlink_nav_cmd_vel_t* nav_cmd_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    nav_cmd_vel->vel_x = mavlink_msg_nav_cmd_vel_get_vel_x(msg);
    nav_cmd_vel->vel_y = mavlink_msg_nav_cmd_vel_get_vel_y(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_nav_cmd_vel_LEN? msg->len : MAVLINK_MSG_ID_nav_cmd_vel_LEN;
        memset(nav_cmd_vel, 0, MAVLINK_MSG_ID_nav_cmd_vel_LEN);
    memcpy(nav_cmd_vel, _MAV_PAYLOAD(msg), len);
#endif
}
