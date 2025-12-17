#pragma once
// MESSAGE aim PACKING

#define MAVLINK_MSG_ID_aim 10


typedef struct __mavlink_aim_t {
 float yaw; /*< [deg] yaw*/
 float pitch; /*< [deg] pitch*/
 uint8_t is_detected; /*<  is_detected*/
 uint8_t is_fire_advise; /*<  is_fire_advise*/
} mavlink_aim_t;

#define MAVLINK_MSG_ID_aim_LEN 10
#define MAVLINK_MSG_ID_aim_MIN_LEN 10
#define MAVLINK_MSG_ID_10_LEN 10
#define MAVLINK_MSG_ID_10_MIN_LEN 10

#define MAVLINK_MSG_ID_aim_CRC 133
#define MAVLINK_MSG_ID_10_CRC 133



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_aim { \
    10, \
    "aim", \
    4, \
    {  { "is_detected", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_aim_t, is_detected) }, \
         { "is_fire_advise", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_aim_t, is_fire_advise) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_aim_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_aim_t, pitch) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_aim { \
    "aim", \
    4, \
    {  { "is_detected", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_aim_t, is_detected) }, \
         { "is_fire_advise", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_aim_t, is_fire_advise) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_aim_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_aim_t, pitch) }, \
         } \
}
#endif

/**
 * @brief Pack a aim message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param is_detected  is_detected
 * @param is_fire_advise  is_fire_advise
 * @param yaw [deg] yaw
 * @param pitch [deg] pitch
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aim_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t is_detected, uint8_t is_fire_advise, float yaw, float pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_aim_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_uint8_t(buf, 8, is_detected);
    _mav_put_uint8_t(buf, 9, is_fire_advise);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_aim_LEN);
#else
    mavlink_aim_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.is_detected = is_detected;
    packet.is_fire_advise = is_fire_advise;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_aim_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_aim;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_aim_MIN_LEN, MAVLINK_MSG_ID_aim_LEN, MAVLINK_MSG_ID_aim_CRC);
}

/**
 * @brief Pack a aim message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param is_detected  is_detected
 * @param is_fire_advise  is_fire_advise
 * @param yaw [deg] yaw
 * @param pitch [deg] pitch
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aim_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t is_detected, uint8_t is_fire_advise, float yaw, float pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_aim_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_uint8_t(buf, 8, is_detected);
    _mav_put_uint8_t(buf, 9, is_fire_advise);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_aim_LEN);
#else
    mavlink_aim_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.is_detected = is_detected;
    packet.is_fire_advise = is_fire_advise;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_aim_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_aim;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_aim_MIN_LEN, MAVLINK_MSG_ID_aim_LEN, MAVLINK_MSG_ID_aim_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_aim_MIN_LEN, MAVLINK_MSG_ID_aim_LEN);
#endif
}

/**
 * @brief Pack a aim message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param is_detected  is_detected
 * @param is_fire_advise  is_fire_advise
 * @param yaw [deg] yaw
 * @param pitch [deg] pitch
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aim_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t is_detected,uint8_t is_fire_advise,float yaw,float pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_aim_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_uint8_t(buf, 8, is_detected);
    _mav_put_uint8_t(buf, 9, is_fire_advise);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_aim_LEN);
#else
    mavlink_aim_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.is_detected = is_detected;
    packet.is_fire_advise = is_fire_advise;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_aim_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_aim;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_aim_MIN_LEN, MAVLINK_MSG_ID_aim_LEN, MAVLINK_MSG_ID_aim_CRC);
}

/**
 * @brief Encode a aim struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param aim C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aim_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_aim_t* aim)
{
    return mavlink_msg_aim_pack(system_id, component_id, msg, aim->is_detected, aim->is_fire_advise, aim->yaw, aim->pitch);
}

/**
 * @brief Encode a aim struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param aim C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aim_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_aim_t* aim)
{
    return mavlink_msg_aim_pack_chan(system_id, component_id, chan, msg, aim->is_detected, aim->is_fire_advise, aim->yaw, aim->pitch);
}

/**
 * @brief Encode a aim struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param aim C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aim_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_aim_t* aim)
{
    return mavlink_msg_aim_pack_status(system_id, component_id, _status, msg,  aim->is_detected, aim->is_fire_advise, aim->yaw, aim->pitch);
}

/**
 * @brief Send a aim message
 * @param chan MAVLink channel to send the message
 *
 * @param is_detected  is_detected
 * @param is_fire_advise  is_fire_advise
 * @param yaw [deg] yaw
 * @param pitch [deg] pitch
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_aim_send(mavlink_channel_t chan, uint8_t is_detected, uint8_t is_fire_advise, float yaw, float pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_aim_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_uint8_t(buf, 8, is_detected);
    _mav_put_uint8_t(buf, 9, is_fire_advise);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_aim, buf, MAVLINK_MSG_ID_aim_MIN_LEN, MAVLINK_MSG_ID_aim_LEN, MAVLINK_MSG_ID_aim_CRC);
#else
    mavlink_aim_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.is_detected = is_detected;
    packet.is_fire_advise = is_fire_advise;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_aim, (const char *)&packet, MAVLINK_MSG_ID_aim_MIN_LEN, MAVLINK_MSG_ID_aim_LEN, MAVLINK_MSG_ID_aim_CRC);
#endif
}

/**
 * @brief Send a aim message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_aim_send_struct(mavlink_channel_t chan, const mavlink_aim_t* aim)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_aim_send(chan, aim->is_detected, aim->is_fire_advise, aim->yaw, aim->pitch);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_aim, (const char *)aim, MAVLINK_MSG_ID_aim_MIN_LEN, MAVLINK_MSG_ID_aim_LEN, MAVLINK_MSG_ID_aim_CRC);
#endif
}

#if MAVLINK_MSG_ID_aim_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_aim_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t is_detected, uint8_t is_fire_advise, float yaw, float pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_uint8_t(buf, 8, is_detected);
    _mav_put_uint8_t(buf, 9, is_fire_advise);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_aim, buf, MAVLINK_MSG_ID_aim_MIN_LEN, MAVLINK_MSG_ID_aim_LEN, MAVLINK_MSG_ID_aim_CRC);
#else
    mavlink_aim_t *packet = (mavlink_aim_t *)msgbuf;
    packet->yaw = yaw;
    packet->pitch = pitch;
    packet->is_detected = is_detected;
    packet->is_fire_advise = is_fire_advise;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_aim, (const char *)packet, MAVLINK_MSG_ID_aim_MIN_LEN, MAVLINK_MSG_ID_aim_LEN, MAVLINK_MSG_ID_aim_CRC);
#endif
}
#endif

#endif

// MESSAGE aim UNPACKING


/**
 * @brief Get field is_detected from aim message
 *
 * @return  is_detected
 */
static inline uint8_t mavlink_msg_aim_get_is_detected(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field is_fire_advise from aim message
 *
 * @return  is_fire_advise
 */
static inline uint8_t mavlink_msg_aim_get_is_fire_advise(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field yaw from aim message
 *
 * @return [deg] yaw
 */
static inline float mavlink_msg_aim_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from aim message
 *
 * @return [deg] pitch
 */
static inline float mavlink_msg_aim_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a aim message into a struct
 *
 * @param msg The message to decode
 * @param aim C-struct to decode the message contents into
 */
static inline void mavlink_msg_aim_decode(const mavlink_message_t* msg, mavlink_aim_t* aim)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    aim->yaw = mavlink_msg_aim_get_yaw(msg);
    aim->pitch = mavlink_msg_aim_get_pitch(msg);
    aim->is_detected = mavlink_msg_aim_get_is_detected(msg);
    aim->is_fire_advise = mavlink_msg_aim_get_is_fire_advise(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_aim_LEN? msg->len : MAVLINK_MSG_ID_aim_LEN;
        memset(aim, 0, MAVLINK_MSG_ID_aim_LEN);
    memcpy(aim, _MAV_PAYLOAD(msg), len);
#endif
}
