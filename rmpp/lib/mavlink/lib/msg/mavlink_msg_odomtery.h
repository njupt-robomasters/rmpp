#pragma once
// MESSAGE odomtery PACKING

#define MAVLINK_MSG_ID_odomtery 22


typedef struct __mavlink_odomtery_t {
 float pos_x; /*< [m] x*/
 float pos_y; /*< [m] y*/
 float yaw; /*< [rad] yaw*/
} mavlink_odomtery_t;

#define MAVLINK_MSG_ID_odomtery_LEN 12
#define MAVLINK_MSG_ID_odomtery_MIN_LEN 12
#define MAVLINK_MSG_ID_22_LEN 12
#define MAVLINK_MSG_ID_22_MIN_LEN 12

#define MAVLINK_MSG_ID_odomtery_CRC 187
#define MAVLINK_MSG_ID_22_CRC 187



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_odomtery { \
    22, \
    "odomtery", \
    3, \
    {  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_odomtery_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_odomtery_t, pos_y) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_odomtery_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_odomtery { \
    "odomtery", \
    3, \
    {  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_odomtery_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_odomtery_t, pos_y) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_odomtery_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a odomtery message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_x [m] x
 * @param pos_y [m] y
 * @param yaw [rad] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odomtery_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pos_x, float pos_y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_odomtery_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_odomtery_LEN);
#else
    mavlink_odomtery_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_odomtery_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_odomtery;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_odomtery_MIN_LEN, MAVLINK_MSG_ID_odomtery_LEN, MAVLINK_MSG_ID_odomtery_CRC);
}

/**
 * @brief Pack a odomtery message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_x [m] x
 * @param pos_y [m] y
 * @param yaw [rad] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odomtery_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float pos_x, float pos_y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_odomtery_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_odomtery_LEN);
#else
    mavlink_odomtery_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_odomtery_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_odomtery;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_odomtery_MIN_LEN, MAVLINK_MSG_ID_odomtery_LEN, MAVLINK_MSG_ID_odomtery_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_odomtery_MIN_LEN, MAVLINK_MSG_ID_odomtery_LEN);
#endif
}

/**
 * @brief Pack a odomtery message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pos_x [m] x
 * @param pos_y [m] y
 * @param yaw [rad] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odomtery_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pos_x,float pos_y,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_odomtery_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_odomtery_LEN);
#else
    mavlink_odomtery_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_odomtery_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_odomtery;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_odomtery_MIN_LEN, MAVLINK_MSG_ID_odomtery_LEN, MAVLINK_MSG_ID_odomtery_CRC);
}

/**
 * @brief Encode a odomtery struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param odomtery C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odomtery_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_odomtery_t* odomtery)
{
    return mavlink_msg_odomtery_pack(system_id, component_id, msg, odomtery->pos_x, odomtery->pos_y, odomtery->yaw);
}

/**
 * @brief Encode a odomtery struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param odomtery C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odomtery_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_odomtery_t* odomtery)
{
    return mavlink_msg_odomtery_pack_chan(system_id, component_id, chan, msg, odomtery->pos_x, odomtery->pos_y, odomtery->yaw);
}

/**
 * @brief Encode a odomtery struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param odomtery C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odomtery_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_odomtery_t* odomtery)
{
    return mavlink_msg_odomtery_pack_status(system_id, component_id, _status, msg,  odomtery->pos_x, odomtery->pos_y, odomtery->yaw);
}

/**
 * @brief Send a odomtery message
 * @param chan MAVLink channel to send the message
 *
 * @param pos_x [m] x
 * @param pos_y [m] y
 * @param yaw [rad] yaw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_odomtery_send(mavlink_channel_t chan, float pos_x, float pos_y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_odomtery_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_odomtery, buf, MAVLINK_MSG_ID_odomtery_MIN_LEN, MAVLINK_MSG_ID_odomtery_LEN, MAVLINK_MSG_ID_odomtery_CRC);
#else
    mavlink_odomtery_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_odomtery, (const char *)&packet, MAVLINK_MSG_ID_odomtery_MIN_LEN, MAVLINK_MSG_ID_odomtery_LEN, MAVLINK_MSG_ID_odomtery_CRC);
#endif
}

/**
 * @brief Send a odomtery message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_odomtery_send_struct(mavlink_channel_t chan, const mavlink_odomtery_t* odomtery)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_odomtery_send(chan, odomtery->pos_x, odomtery->pos_y, odomtery->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_odomtery, (const char *)odomtery, MAVLINK_MSG_ID_odomtery_MIN_LEN, MAVLINK_MSG_ID_odomtery_LEN, MAVLINK_MSG_ID_odomtery_CRC);
#endif
}

#if MAVLINK_MSG_ID_odomtery_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_odomtery_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pos_x, float pos_y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_odomtery, buf, MAVLINK_MSG_ID_odomtery_MIN_LEN, MAVLINK_MSG_ID_odomtery_LEN, MAVLINK_MSG_ID_odomtery_CRC);
#else
    mavlink_odomtery_t *packet = (mavlink_odomtery_t *)msgbuf;
    packet->pos_x = pos_x;
    packet->pos_y = pos_y;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_odomtery, (const char *)packet, MAVLINK_MSG_ID_odomtery_MIN_LEN, MAVLINK_MSG_ID_odomtery_LEN, MAVLINK_MSG_ID_odomtery_CRC);
#endif
}
#endif

#endif

// MESSAGE odomtery UNPACKING


/**
 * @brief Get field pos_x from odomtery message
 *
 * @return [m] x
 */
static inline float mavlink_msg_odomtery_get_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pos_y from odomtery message
 *
 * @return [m] y
 */
static inline float mavlink_msg_odomtery_get_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from odomtery message
 *
 * @return [rad] yaw
 */
static inline float mavlink_msg_odomtery_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a odomtery message into a struct
 *
 * @param msg The message to decode
 * @param odomtery C-struct to decode the message contents into
 */
static inline void mavlink_msg_odomtery_decode(const mavlink_message_t* msg, mavlink_odomtery_t* odomtery)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    odomtery->pos_x = mavlink_msg_odomtery_get_pos_x(msg);
    odomtery->pos_y = mavlink_msg_odomtery_get_pos_y(msg);
    odomtery->yaw = mavlink_msg_odomtery_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_odomtery_LEN? msg->len : MAVLINK_MSG_ID_odomtery_LEN;
        memset(odomtery, 0, MAVLINK_MSG_ID_odomtery_LEN);
    memcpy(odomtery, _MAV_PAYLOAD(msg), len);
#endif
}
