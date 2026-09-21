#pragma once
// MESSAGE chassis_speed PACKING

#define MAVLINK_MSG_ID_chassis_speed 6


typedef struct __mavlink_chassis_speed_t {
 float vx; /*< [m/s] x*/
 float vy; /*< [m/s] y*/
} mavlink_chassis_speed_t;

#define MAVLINK_MSG_ID_chassis_speed_LEN 8
#define MAVLINK_MSG_ID_chassis_speed_MIN_LEN 8
#define MAVLINK_MSG_ID_6_LEN 8
#define MAVLINK_MSG_ID_6_MIN_LEN 8

#define MAVLINK_MSG_ID_chassis_speed_CRC 207
#define MAVLINK_MSG_ID_6_CRC 207



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_chassis_speed { \
    6, \
    "chassis_speed", \
    2, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chassis_speed_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chassis_speed_t, vy) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_chassis_speed { \
    "chassis_speed", \
    2, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chassis_speed_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chassis_speed_t, vy) }, \
         } \
}
#endif

/**
 * @brief Pack a chassis_speed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] x
 * @param vy [m/s] y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chassis_speed_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float vx, float vy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_chassis_speed_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_chassis_speed_LEN);
#else
    mavlink_chassis_speed_t packet;
    packet.vx = vx;
    packet.vy = vy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_chassis_speed_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_chassis_speed;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_chassis_speed_MIN_LEN, MAVLINK_MSG_ID_chassis_speed_LEN, MAVLINK_MSG_ID_chassis_speed_CRC);
}

/**
 * @brief Pack a chassis_speed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] x
 * @param vy [m/s] y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chassis_speed_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float vx, float vy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_chassis_speed_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_chassis_speed_LEN);
#else
    mavlink_chassis_speed_t packet;
    packet.vx = vx;
    packet.vy = vy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_chassis_speed_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_chassis_speed;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_chassis_speed_MIN_LEN, MAVLINK_MSG_ID_chassis_speed_LEN, MAVLINK_MSG_ID_chassis_speed_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_chassis_speed_MIN_LEN, MAVLINK_MSG_ID_chassis_speed_LEN);
#endif
}

/**
 * @brief Pack a chassis_speed message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vx [m/s] x
 * @param vy [m/s] y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chassis_speed_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float vx,float vy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_chassis_speed_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_chassis_speed_LEN);
#else
    mavlink_chassis_speed_t packet;
    packet.vx = vx;
    packet.vy = vy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_chassis_speed_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_chassis_speed;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_chassis_speed_MIN_LEN, MAVLINK_MSG_ID_chassis_speed_LEN, MAVLINK_MSG_ID_chassis_speed_CRC);
}

/**
 * @brief Encode a chassis_speed struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chassis_speed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chassis_speed_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chassis_speed_t* chassis_speed)
{
    return mavlink_msg_chassis_speed_pack(system_id, component_id, msg, chassis_speed->vx, chassis_speed->vy);
}

/**
 * @brief Encode a chassis_speed struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chassis_speed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chassis_speed_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chassis_speed_t* chassis_speed)
{
    return mavlink_msg_chassis_speed_pack_chan(system_id, component_id, chan, msg, chassis_speed->vx, chassis_speed->vy);
}

/**
 * @brief Encode a chassis_speed struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param chassis_speed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chassis_speed_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_chassis_speed_t* chassis_speed)
{
    return mavlink_msg_chassis_speed_pack_status(system_id, component_id, _status, msg,  chassis_speed->vx, chassis_speed->vy);
}

/**
 * @brief Send a chassis_speed message
 * @param chan MAVLink channel to send the message
 *
 * @param vx [m/s] x
 * @param vy [m/s] y
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chassis_speed_send(mavlink_channel_t chan, float vx, float vy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_chassis_speed_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_chassis_speed, buf, MAVLINK_MSG_ID_chassis_speed_MIN_LEN, MAVLINK_MSG_ID_chassis_speed_LEN, MAVLINK_MSG_ID_chassis_speed_CRC);
#else
    mavlink_chassis_speed_t packet;
    packet.vx = vx;
    packet.vy = vy;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_chassis_speed, (const char *)&packet, MAVLINK_MSG_ID_chassis_speed_MIN_LEN, MAVLINK_MSG_ID_chassis_speed_LEN, MAVLINK_MSG_ID_chassis_speed_CRC);
#endif
}

/**
 * @brief Send a chassis_speed message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_chassis_speed_send_struct(mavlink_channel_t chan, const mavlink_chassis_speed_t* chassis_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chassis_speed_send(chan, chassis_speed->vx, chassis_speed->vy);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_chassis_speed, (const char *)chassis_speed, MAVLINK_MSG_ID_chassis_speed_MIN_LEN, MAVLINK_MSG_ID_chassis_speed_LEN, MAVLINK_MSG_ID_chassis_speed_CRC);
#endif
}

#if MAVLINK_MSG_ID_chassis_speed_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chassis_speed_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vx, float vy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_chassis_speed, buf, MAVLINK_MSG_ID_chassis_speed_MIN_LEN, MAVLINK_MSG_ID_chassis_speed_LEN, MAVLINK_MSG_ID_chassis_speed_CRC);
#else
    mavlink_chassis_speed_t *packet = (mavlink_chassis_speed_t *)msgbuf;
    packet->vx = vx;
    packet->vy = vy;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_chassis_speed, (const char *)packet, MAVLINK_MSG_ID_chassis_speed_MIN_LEN, MAVLINK_MSG_ID_chassis_speed_LEN, MAVLINK_MSG_ID_chassis_speed_CRC);
#endif
}
#endif

#endif

// MESSAGE chassis_speed UNPACKING


/**
 * @brief Get field vx from chassis_speed message
 *
 * @return [m/s] x
 */
static inline float mavlink_msg_chassis_speed_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vy from chassis_speed message
 *
 * @return [m/s] y
 */
static inline float mavlink_msg_chassis_speed_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a chassis_speed message into a struct
 *
 * @param msg The message to decode
 * @param chassis_speed C-struct to decode the message contents into
 */
static inline void mavlink_msg_chassis_speed_decode(const mavlink_message_t* msg, mavlink_chassis_speed_t* chassis_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    chassis_speed->vx = mavlink_msg_chassis_speed_get_vx(msg);
    chassis_speed->vy = mavlink_msg_chassis_speed_get_vy(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_chassis_speed_LEN? msg->len : MAVLINK_MSG_ID_chassis_speed_LEN;
        memset(chassis_speed, 0, MAVLINK_MSG_ID_chassis_speed_LEN);
    memcpy(chassis_speed, _MAV_PAYLOAD(msg), len);
#endif
}
