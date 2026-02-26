#pragma once
// MESSAGE target_pose PACKING

#define MAVLINK_MSG_ID_target_pose 21


typedef struct __mavlink_target_pose_t {
 float pos_x; /*< [m] x*/
 float pos_y; /*< [m] y*/
} mavlink_target_pose_t;

#define MAVLINK_MSG_ID_target_pose_LEN 8
#define MAVLINK_MSG_ID_target_pose_MIN_LEN 8
#define MAVLINK_MSG_ID_21_LEN 8
#define MAVLINK_MSG_ID_21_MIN_LEN 8

#define MAVLINK_MSG_ID_target_pose_CRC 63
#define MAVLINK_MSG_ID_21_CRC 63



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_target_pose { \
    21, \
    "target_pose", \
    2, \
    {  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_target_pose_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_pose_t, pos_y) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_target_pose { \
    "target_pose", \
    2, \
    {  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_target_pose_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_pose_t, pos_y) }, \
         } \
}
#endif

/**
 * @brief Pack a target_pose message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_x [m] x
 * @param pos_y [m] y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_pose_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pos_x, float pos_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_target_pose_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_target_pose_LEN);
#else
    mavlink_target_pose_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_target_pose_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_target_pose;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_target_pose_MIN_LEN, MAVLINK_MSG_ID_target_pose_LEN, MAVLINK_MSG_ID_target_pose_CRC);
}

/**
 * @brief Pack a target_pose message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_x [m] x
 * @param pos_y [m] y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_pose_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float pos_x, float pos_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_target_pose_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_target_pose_LEN);
#else
    mavlink_target_pose_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_target_pose_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_target_pose;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_target_pose_MIN_LEN, MAVLINK_MSG_ID_target_pose_LEN, MAVLINK_MSG_ID_target_pose_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_target_pose_MIN_LEN, MAVLINK_MSG_ID_target_pose_LEN);
#endif
}

/**
 * @brief Pack a target_pose message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pos_x [m] x
 * @param pos_y [m] y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_pose_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pos_x,float pos_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_target_pose_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_target_pose_LEN);
#else
    mavlink_target_pose_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_target_pose_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_target_pose;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_target_pose_MIN_LEN, MAVLINK_MSG_ID_target_pose_LEN, MAVLINK_MSG_ID_target_pose_CRC);
}

/**
 * @brief Encode a target_pose struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param target_pose C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_pose_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_target_pose_t* target_pose)
{
    return mavlink_msg_target_pose_pack(system_id, component_id, msg, target_pose->pos_x, target_pose->pos_y);
}

/**
 * @brief Encode a target_pose struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_pose C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_pose_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_target_pose_t* target_pose)
{
    return mavlink_msg_target_pose_pack_chan(system_id, component_id, chan, msg, target_pose->pos_x, target_pose->pos_y);
}

/**
 * @brief Encode a target_pose struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param target_pose C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_pose_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_target_pose_t* target_pose)
{
    return mavlink_msg_target_pose_pack_status(system_id, component_id, _status, msg,  target_pose->pos_x, target_pose->pos_y);
}

/**
 * @brief Send a target_pose message
 * @param chan MAVLink channel to send the message
 *
 * @param pos_x [m] x
 * @param pos_y [m] y
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_target_pose_send(mavlink_channel_t chan, float pos_x, float pos_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_target_pose_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_target_pose, buf, MAVLINK_MSG_ID_target_pose_MIN_LEN, MAVLINK_MSG_ID_target_pose_LEN, MAVLINK_MSG_ID_target_pose_CRC);
#else
    mavlink_target_pose_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_target_pose, (const char *)&packet, MAVLINK_MSG_ID_target_pose_MIN_LEN, MAVLINK_MSG_ID_target_pose_LEN, MAVLINK_MSG_ID_target_pose_CRC);
#endif
}

/**
 * @brief Send a target_pose message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_target_pose_send_struct(mavlink_channel_t chan, const mavlink_target_pose_t* target_pose)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_target_pose_send(chan, target_pose->pos_x, target_pose->pos_y);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_target_pose, (const char *)target_pose, MAVLINK_MSG_ID_target_pose_MIN_LEN, MAVLINK_MSG_ID_target_pose_LEN, MAVLINK_MSG_ID_target_pose_CRC);
#endif
}

#if MAVLINK_MSG_ID_target_pose_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_target_pose_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pos_x, float pos_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_target_pose, buf, MAVLINK_MSG_ID_target_pose_MIN_LEN, MAVLINK_MSG_ID_target_pose_LEN, MAVLINK_MSG_ID_target_pose_CRC);
#else
    mavlink_target_pose_t *packet = (mavlink_target_pose_t *)msgbuf;
    packet->pos_x = pos_x;
    packet->pos_y = pos_y;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_target_pose, (const char *)packet, MAVLINK_MSG_ID_target_pose_MIN_LEN, MAVLINK_MSG_ID_target_pose_LEN, MAVLINK_MSG_ID_target_pose_CRC);
#endif
}
#endif

#endif

// MESSAGE target_pose UNPACKING


/**
 * @brief Get field pos_x from target_pose message
 *
 * @return [m] x
 */
static inline float mavlink_msg_target_pose_get_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pos_y from target_pose message
 *
 * @return [m] y
 */
static inline float mavlink_msg_target_pose_get_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a target_pose message into a struct
 *
 * @param msg The message to decode
 * @param target_pose C-struct to decode the message contents into
 */
static inline void mavlink_msg_target_pose_decode(const mavlink_message_t* msg, mavlink_target_pose_t* target_pose)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    target_pose->pos_x = mavlink_msg_target_pose_get_pos_x(msg);
    target_pose->pos_y = mavlink_msg_target_pose_get_pos_y(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_target_pose_LEN? msg->len : MAVLINK_MSG_ID_target_pose_LEN;
        memset(target_pose, 0, MAVLINK_MSG_ID_target_pose_LEN);
    memcpy(target_pose, _MAV_PAYLOAD(msg), len);
#endif
}
