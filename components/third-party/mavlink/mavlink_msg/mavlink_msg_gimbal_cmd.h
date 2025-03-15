#pragma once
// MESSAGE GIMBAL_CMD PACKING

#define MAVLINK_MSG_ID_GIMBAL_CMD 1


typedef struct __mavlink_gimbal_cmd_t {
 float pitch; /*< [deg] pitch*/
 float yaw; /*< [deg] yaw*/
 uint8_t fire_advise; /*<  to control fire*/
} mavlink_gimbal_cmd_t;

#define MAVLINK_MSG_ID_GIMBAL_CMD_LEN 9
#define MAVLINK_MSG_ID_GIMBAL_CMD_MIN_LEN 9
#define MAVLINK_MSG_ID_1_LEN 9
#define MAVLINK_MSG_ID_1_MIN_LEN 9

#define MAVLINK_MSG_ID_GIMBAL_CMD_CRC 88
#define MAVLINK_MSG_ID_1_CRC 88



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_CMD { \
    1, \
    "GIMBAL_CMD", \
    3, \
    {  { "fire_advise", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gimbal_cmd_t, fire_advise) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_cmd_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_cmd_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_CMD { \
    "GIMBAL_CMD", \
    3, \
    {  { "fire_advise", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gimbal_cmd_t, fire_advise) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_cmd_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_cmd_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param fire_advise  to control fire
 * @param pitch [deg] pitch
 * @param yaw [deg] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t fire_advise, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_CMD_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_uint8_t(buf, 8, fire_advise);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_CMD_LEN);
#else
    mavlink_gimbal_cmd_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.fire_advise = fire_advise;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_CMD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_CRC);
}

/**
 * @brief Pack a gimbal_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param fire_advise  to control fire
 * @param pitch [deg] pitch
 * @param yaw [deg] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_cmd_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t fire_advise, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_CMD_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_uint8_t(buf, 8, fire_advise);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_CMD_LEN);
#else
    mavlink_gimbal_cmd_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.fire_advise = fire_advise;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GIMBAL_CMD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GIMBAL_CMD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_LEN);
#endif
}

/**
 * @brief Pack a gimbal_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fire_advise  to control fire
 * @param pitch [deg] pitch
 * @param yaw [deg] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t fire_advise,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_CMD_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_uint8_t(buf, 8, fire_advise);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_CMD_LEN);
#else
    mavlink_gimbal_cmd_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.fire_advise = fire_advise;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_CMD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_CRC);
}

/**
 * @brief Encode a gimbal_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_cmd_t* gimbal_cmd)
{
    return mavlink_msg_gimbal_cmd_pack(system_id, component_id, msg, gimbal_cmd->fire_advise, gimbal_cmd->pitch, gimbal_cmd->yaw);
}

/**
 * @brief Encode a gimbal_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_cmd_t* gimbal_cmd)
{
    return mavlink_msg_gimbal_cmd_pack_chan(system_id, component_id, chan, msg, gimbal_cmd->fire_advise, gimbal_cmd->pitch, gimbal_cmd->yaw);
}

/**
 * @brief Encode a gimbal_cmd struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_cmd_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gimbal_cmd_t* gimbal_cmd)
{
    return mavlink_msg_gimbal_cmd_pack_status(system_id, component_id, _status, msg,  gimbal_cmd->fire_advise, gimbal_cmd->pitch, gimbal_cmd->yaw);
}

/**
 * @brief Send a gimbal_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param fire_advise  to control fire
 * @param pitch [deg] pitch
 * @param yaw [deg] yaw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_cmd_send(mavlink_channel_t chan, uint8_t fire_advise, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_CMD_LEN];
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_uint8_t(buf, 8, fire_advise);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CMD, buf, MAVLINK_MSG_ID_GIMBAL_CMD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_CRC);
#else
    mavlink_gimbal_cmd_t packet;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.fire_advise = fire_advise;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CMD, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_CMD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_CRC);
#endif
}

/**
 * @brief Send a gimbal_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_cmd_send_struct(mavlink_channel_t chan, const mavlink_gimbal_cmd_t* gimbal_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_cmd_send(chan, gimbal_cmd->fire_advise, gimbal_cmd->pitch, gimbal_cmd->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CMD, (const char *)gimbal_cmd, MAVLINK_MSG_ID_GIMBAL_CMD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t fire_advise, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pitch);
    _mav_put_float(buf, 4, yaw);
    _mav_put_uint8_t(buf, 8, fire_advise);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CMD, buf, MAVLINK_MSG_ID_GIMBAL_CMD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_CRC);
#else
    mavlink_gimbal_cmd_t *packet = (mavlink_gimbal_cmd_t *)msgbuf;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->fire_advise = fire_advise;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_CMD, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_CMD_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_LEN, MAVLINK_MSG_ID_GIMBAL_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_CMD UNPACKING


/**
 * @brief Get field fire_advise from gimbal_cmd message
 *
 * @return  to control fire
 */
static inline uint8_t mavlink_msg_gimbal_cmd_get_fire_advise(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field pitch from gimbal_cmd message
 *
 * @return [deg] pitch
 */
static inline float mavlink_msg_gimbal_cmd_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field yaw from gimbal_cmd message
 *
 * @return [deg] yaw
 */
static inline float mavlink_msg_gimbal_cmd_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a gimbal_cmd message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_cmd_decode(const mavlink_message_t* msg, mavlink_gimbal_cmd_t* gimbal_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_cmd->pitch = mavlink_msg_gimbal_cmd_get_pitch(msg);
    gimbal_cmd->yaw = mavlink_msg_gimbal_cmd_get_yaw(msg);
    gimbal_cmd->fire_advise = mavlink_msg_gimbal_cmd_get_fire_advise(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_CMD_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_CMD_LEN;
        memset(gimbal_cmd, 0, MAVLINK_MSG_ID_GIMBAL_CMD_LEN);
    memcpy(gimbal_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
