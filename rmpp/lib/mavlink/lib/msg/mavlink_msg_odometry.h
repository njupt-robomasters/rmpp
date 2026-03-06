#pragma once
// MESSAGE odometry PACKING

#define MAVLINK_MSG_ID_odometry 22


typedef struct __mavlink_odometry_t {
 float pos_x; /*< [m] x*/
 float pos_y; /*< [m] y*/
 float yaw; /*< [rad] yaw*/
} mavlink_odometry_t;

#define MAVLINK_MSG_ID_odometry_LEN 12
#define MAVLINK_MSG_ID_odometry_MIN_LEN 12
#define MAVLINK_MSG_ID_22_LEN 12
#define MAVLINK_MSG_ID_22_MIN_LEN 12

#define MAVLINK_MSG_ID_odometry_CRC 129
#define MAVLINK_MSG_ID_22_CRC 129



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_odometry { \
    22, \
    "odometry", \
    3, \
    {  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_odometry_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_odometry_t, pos_y) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_odometry_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_odometry { \
    "odometry", \
    3, \
    {  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_odometry_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_odometry_t, pos_y) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_odometry_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a odometry message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_x [m] x
 * @param pos_y [m] y
 * @param yaw [rad] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometry_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pos_x, float pos_y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_odometry_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_odometry_LEN);
#else
    mavlink_odometry_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_odometry_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_odometry;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_odometry_MIN_LEN, MAVLINK_MSG_ID_odometry_LEN, MAVLINK_MSG_ID_odometry_CRC);
}

/**
 * @brief Pack a odometry message
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
static inline uint16_t mavlink_msg_odometry_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float pos_x, float pos_y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_odometry_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_odometry_LEN);
#else
    mavlink_odometry_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_odometry_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_odometry;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_odometry_MIN_LEN, MAVLINK_MSG_ID_odometry_LEN, MAVLINK_MSG_ID_odometry_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_odometry_MIN_LEN, MAVLINK_MSG_ID_odometry_LEN);
#endif
}

/**
 * @brief Pack a odometry message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pos_x [m] x
 * @param pos_y [m] y
 * @param yaw [rad] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odometry_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pos_x,float pos_y,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_odometry_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_odometry_LEN);
#else
    mavlink_odometry_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_odometry_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_odometry;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_odometry_MIN_LEN, MAVLINK_MSG_ID_odometry_LEN, MAVLINK_MSG_ID_odometry_CRC);
}

/**
 * @brief Encode a odometry struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param odometry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometry_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_odometry_t* odometry)
{
    return mavlink_msg_odometry_pack(system_id, component_id, msg, odometry->pos_x, odometry->pos_y, odometry->yaw);
}

/**
 * @brief Encode a odometry struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param odometry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometry_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_odometry_t* odometry)
{
    return mavlink_msg_odometry_pack_chan(system_id, component_id, chan, msg, odometry->pos_x, odometry->pos_y, odometry->yaw);
}

/**
 * @brief Encode a odometry struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param odometry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odometry_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_odometry_t* odometry)
{
    return mavlink_msg_odometry_pack_status(system_id, component_id, _status, msg,  odometry->pos_x, odometry->pos_y, odometry->yaw);
}

/**
 * @brief Send a odometry message
 * @param chan MAVLink channel to send the message
 *
 * @param pos_x [m] x
 * @param pos_y [m] y
 * @param yaw [rad] yaw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_odometry_send(mavlink_channel_t chan, float pos_x, float pos_y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_odometry_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_odometry, buf, MAVLINK_MSG_ID_odometry_MIN_LEN, MAVLINK_MSG_ID_odometry_LEN, MAVLINK_MSG_ID_odometry_CRC);
#else
    mavlink_odometry_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_odometry, (const char *)&packet, MAVLINK_MSG_ID_odometry_MIN_LEN, MAVLINK_MSG_ID_odometry_LEN, MAVLINK_MSG_ID_odometry_CRC);
#endif
}

/**
 * @brief Send a odometry message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_odometry_send_struct(mavlink_channel_t chan, const mavlink_odometry_t* odometry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_odometry_send(chan, odometry->pos_x, odometry->pos_y, odometry->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_odometry, (const char *)odometry, MAVLINK_MSG_ID_odometry_MIN_LEN, MAVLINK_MSG_ID_odometry_LEN, MAVLINK_MSG_ID_odometry_CRC);
#endif
}

#if MAVLINK_MSG_ID_odometry_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_odometry_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pos_x, float pos_y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_odometry, buf, MAVLINK_MSG_ID_odometry_MIN_LEN, MAVLINK_MSG_ID_odometry_LEN, MAVLINK_MSG_ID_odometry_CRC);
#else
    mavlink_odometry_t *packet = (mavlink_odometry_t *)msgbuf;
    packet->pos_x = pos_x;
    packet->pos_y = pos_y;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_odometry, (const char *)packet, MAVLINK_MSG_ID_odometry_MIN_LEN, MAVLINK_MSG_ID_odometry_LEN, MAVLINK_MSG_ID_odometry_CRC);
#endif
}
#endif

#endif

// MESSAGE odometry UNPACKING


/**
 * @brief Get field pos_x from odometry message
 *
 * @return [m] x
 */
static inline float mavlink_msg_odometry_get_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pos_y from odometry message
 *
 * @return [m] y
 */
static inline float mavlink_msg_odometry_get_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from odometry message
 *
 * @return [rad] yaw
 */
static inline float mavlink_msg_odometry_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a odometry message into a struct
 *
 * @param msg The message to decode
 * @param odometry C-struct to decode the message contents into
 */
static inline void mavlink_msg_odometry_decode(const mavlink_message_t* msg, mavlink_odometry_t* odometry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    odometry->pos_x = mavlink_msg_odometry_get_pos_x(msg);
    odometry->pos_y = mavlink_msg_odometry_get_pos_y(msg);
    odometry->yaw = mavlink_msg_odometry_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_odometry_LEN? msg->len : MAVLINK_MSG_ID_odometry_LEN;
        memset(odometry, 0, MAVLINK_MSG_ID_odometry_LEN);
    memcpy(odometry, _MAV_PAYLOAD(msg), len);
#endif
}
