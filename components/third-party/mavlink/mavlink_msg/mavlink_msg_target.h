#pragma once
// MESSAGE TARGET PACKING

#define MAVLINK_MSG_ID_TARGET 9


typedef struct __mavlink_target_t {
 float x_target; /*< [m] x_target*/
 float t_target; /*< [m] x_target*/
} mavlink_target_t;

#define MAVLINK_MSG_ID_TARGET_LEN 8
#define MAVLINK_MSG_ID_TARGET_MIN_LEN 8
#define MAVLINK_MSG_ID_9_LEN 8
#define MAVLINK_MSG_ID_9_MIN_LEN 8

#define MAVLINK_MSG_ID_TARGET_CRC 8
#define MAVLINK_MSG_ID_9_CRC 8



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TARGET { \
    9, \
    "TARGET", \
    2, \
    {  { "x_target", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_target_t, x_target) }, \
         { "t_target", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_t, t_target) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TARGET { \
    "TARGET", \
    2, \
    {  { "x_target", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_target_t, x_target) }, \
         { "t_target", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_t, t_target) }, \
         } \
}
#endif

/**
 * @brief Pack a target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x_target [m] x_target
 * @param t_target [m] x_target
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x_target, float t_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_LEN];
    _mav_put_float(buf, 0, x_target);
    _mav_put_float(buf, 4, t_target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_LEN);
#else
    mavlink_target_t packet;
    packet.x_target = x_target;
    packet.t_target = t_target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGET_MIN_LEN, MAVLINK_MSG_ID_TARGET_LEN, MAVLINK_MSG_ID_TARGET_CRC);
}

/**
 * @brief Pack a target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param x_target [m] x_target
 * @param t_target [m] x_target
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float x_target, float t_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_LEN];
    _mav_put_float(buf, 0, x_target);
    _mav_put_float(buf, 4, t_target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_LEN);
#else
    mavlink_target_t packet;
    packet.x_target = x_target;
    packet.t_target = t_target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TARGET_MIN_LEN, MAVLINK_MSG_ID_TARGET_LEN, MAVLINK_MSG_ID_TARGET_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TARGET_MIN_LEN, MAVLINK_MSG_ID_TARGET_LEN);
#endif
}

/**
 * @brief Pack a target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x_target [m] x_target
 * @param t_target [m] x_target
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x_target,float t_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_LEN];
    _mav_put_float(buf, 0, x_target);
    _mav_put_float(buf, 4, t_target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_LEN);
#else
    mavlink_target_t packet;
    packet.x_target = x_target;
    packet.t_target = t_target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGET_MIN_LEN, MAVLINK_MSG_ID_TARGET_LEN, MAVLINK_MSG_ID_TARGET_CRC);
}

/**
 * @brief Encode a target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_target_t* target)
{
    return mavlink_msg_target_pack(system_id, component_id, msg, target->x_target, target->t_target);
}

/**
 * @brief Encode a target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_target_t* target)
{
    return mavlink_msg_target_pack_chan(system_id, component_id, chan, msg, target->x_target, target->t_target);
}

/**
 * @brief Encode a target struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_target_t* target)
{
    return mavlink_msg_target_pack_status(system_id, component_id, _status, msg,  target->x_target, target->t_target);
}

/**
 * @brief Send a target message
 * @param chan MAVLink channel to send the message
 *
 * @param x_target [m] x_target
 * @param t_target [m] x_target
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_target_send(mavlink_channel_t chan, float x_target, float t_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_LEN];
    _mav_put_float(buf, 0, x_target);
    _mav_put_float(buf, 4, t_target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET, buf, MAVLINK_MSG_ID_TARGET_MIN_LEN, MAVLINK_MSG_ID_TARGET_LEN, MAVLINK_MSG_ID_TARGET_CRC);
#else
    mavlink_target_t packet;
    packet.x_target = x_target;
    packet.t_target = t_target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET, (const char *)&packet, MAVLINK_MSG_ID_TARGET_MIN_LEN, MAVLINK_MSG_ID_TARGET_LEN, MAVLINK_MSG_ID_TARGET_CRC);
#endif
}

/**
 * @brief Send a target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_target_send_struct(mavlink_channel_t chan, const mavlink_target_t* target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_target_send(chan, target->x_target, target->t_target);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET, (const char *)target, MAVLINK_MSG_ID_TARGET_MIN_LEN, MAVLINK_MSG_ID_TARGET_LEN, MAVLINK_MSG_ID_TARGET_CRC);
#endif
}

#if MAVLINK_MSG_ID_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x_target, float t_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x_target);
    _mav_put_float(buf, 4, t_target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET, buf, MAVLINK_MSG_ID_TARGET_MIN_LEN, MAVLINK_MSG_ID_TARGET_LEN, MAVLINK_MSG_ID_TARGET_CRC);
#else
    mavlink_target_t *packet = (mavlink_target_t *)msgbuf;
    packet->x_target = x_target;
    packet->t_target = t_target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET, (const char *)packet, MAVLINK_MSG_ID_TARGET_MIN_LEN, MAVLINK_MSG_ID_TARGET_LEN, MAVLINK_MSG_ID_TARGET_CRC);
#endif
}
#endif

#endif

// MESSAGE TARGET UNPACKING


/**
 * @brief Get field x_target from target message
 *
 * @return [m] x_target
 */
static inline float mavlink_msg_target_get_x_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field t_target from target message
 *
 * @return [m] x_target
 */
static inline float mavlink_msg_target_get_t_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a target message into a struct
 *
 * @param msg The message to decode
 * @param target C-struct to decode the message contents into
 */
static inline void mavlink_msg_target_decode(const mavlink_message_t* msg, mavlink_target_t* target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    target->x_target = mavlink_msg_target_get_x_target(msg);
    target->t_target = mavlink_msg_target_get_t_target(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TARGET_LEN? msg->len : MAVLINK_MSG_ID_TARGET_LEN;
        memset(target, 0, MAVLINK_MSG_ID_TARGET_LEN);
    memcpy(target, _MAV_PAYLOAD(msg), len);
#endif
}
