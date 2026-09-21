#pragma once
// MESSAGE auto_aim PACKING

#define MAVLINK_MSG_ID_auto_aim 2


typedef struct __mavlink_auto_aim_t {
 float yaw; /*< [deg] yaw(left is positive)*/
 float pitch; /*< [deg] pitch(up is positive)*/
 float wr; /*< [rpm] rpm*/
 float distance; /*< [m] distance*/
 uint8_t is_detect; /*<  is_detect*/
 uint8_t is_fire; /*<  is_fire*/
 uint8_t robot_id; /*<  robot_id(1-7, see robomaster manual)*/
} mavlink_auto_aim_t;

#define MAVLINK_MSG_ID_auto_aim_LEN 19
#define MAVLINK_MSG_ID_auto_aim_MIN_LEN 19
#define MAVLINK_MSG_ID_2_LEN 19
#define MAVLINK_MSG_ID_2_MIN_LEN 19

#define MAVLINK_MSG_ID_auto_aim_CRC 152
#define MAVLINK_MSG_ID_2_CRC 152



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_auto_aim { \
    2, \
    "auto_aim", \
    7, \
    {  { "is_detect", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_auto_aim_t, is_detect) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_auto_aim_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_auto_aim_t, pitch) }, \
         { "is_fire", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_auto_aim_t, is_fire) }, \
         { "robot_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_auto_aim_t, robot_id) }, \
         { "wr", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_auto_aim_t, wr) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_auto_aim_t, distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_auto_aim { \
    "auto_aim", \
    7, \
    {  { "is_detect", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_auto_aim_t, is_detect) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_auto_aim_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_auto_aim_t, pitch) }, \
         { "is_fire", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_auto_aim_t, is_fire) }, \
         { "robot_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_auto_aim_t, robot_id) }, \
         { "wr", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_auto_aim_t, wr) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_auto_aim_t, distance) }, \
         } \
}
#endif

/**
 * @brief Pack a auto_aim message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param is_detect  is_detect
 * @param yaw [deg] yaw(left is positive)
 * @param pitch [deg] pitch(up is positive)
 * @param is_fire  is_fire
 * @param robot_id  robot_id(1-7, see robomaster manual)
 * @param wr [rpm] rpm
 * @param distance [m] distance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_aim_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t is_detect, float yaw, float pitch, uint8_t is_fire, uint8_t robot_id, float wr, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_auto_aim_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, wr);
    _mav_put_float(buf, 12, distance);
    _mav_put_uint8_t(buf, 16, is_detect);
    _mav_put_uint8_t(buf, 17, is_fire);
    _mav_put_uint8_t(buf, 18, robot_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_auto_aim_LEN);
#else
    mavlink_auto_aim_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.wr = wr;
    packet.distance = distance;
    packet.is_detect = is_detect;
    packet.is_fire = is_fire;
    packet.robot_id = robot_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_auto_aim_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_auto_aim;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_auto_aim_MIN_LEN, MAVLINK_MSG_ID_auto_aim_LEN, MAVLINK_MSG_ID_auto_aim_CRC);
}

/**
 * @brief Pack a auto_aim message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param is_detect  is_detect
 * @param yaw [deg] yaw(left is positive)
 * @param pitch [deg] pitch(up is positive)
 * @param is_fire  is_fire
 * @param robot_id  robot_id(1-7, see robomaster manual)
 * @param wr [rpm] rpm
 * @param distance [m] distance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_aim_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t is_detect, float yaw, float pitch, uint8_t is_fire, uint8_t robot_id, float wr, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_auto_aim_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, wr);
    _mav_put_float(buf, 12, distance);
    _mav_put_uint8_t(buf, 16, is_detect);
    _mav_put_uint8_t(buf, 17, is_fire);
    _mav_put_uint8_t(buf, 18, robot_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_auto_aim_LEN);
#else
    mavlink_auto_aim_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.wr = wr;
    packet.distance = distance;
    packet.is_detect = is_detect;
    packet.is_fire = is_fire;
    packet.robot_id = robot_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_auto_aim_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_auto_aim;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_auto_aim_MIN_LEN, MAVLINK_MSG_ID_auto_aim_LEN, MAVLINK_MSG_ID_auto_aim_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_auto_aim_MIN_LEN, MAVLINK_MSG_ID_auto_aim_LEN);
#endif
}

/**
 * @brief Pack a auto_aim message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param is_detect  is_detect
 * @param yaw [deg] yaw(left is positive)
 * @param pitch [deg] pitch(up is positive)
 * @param is_fire  is_fire
 * @param robot_id  robot_id(1-7, see robomaster manual)
 * @param wr [rpm] rpm
 * @param distance [m] distance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auto_aim_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t is_detect,float yaw,float pitch,uint8_t is_fire,uint8_t robot_id,float wr,float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_auto_aim_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, wr);
    _mav_put_float(buf, 12, distance);
    _mav_put_uint8_t(buf, 16, is_detect);
    _mav_put_uint8_t(buf, 17, is_fire);
    _mav_put_uint8_t(buf, 18, robot_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_auto_aim_LEN);
#else
    mavlink_auto_aim_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.wr = wr;
    packet.distance = distance;
    packet.is_detect = is_detect;
    packet.is_fire = is_fire;
    packet.robot_id = robot_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_auto_aim_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_auto_aim;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_auto_aim_MIN_LEN, MAVLINK_MSG_ID_auto_aim_LEN, MAVLINK_MSG_ID_auto_aim_CRC);
}

/**
 * @brief Encode a auto_aim struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auto_aim C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_aim_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_auto_aim_t* auto_aim)
{
    return mavlink_msg_auto_aim_pack(system_id, component_id, msg, auto_aim->is_detect, auto_aim->yaw, auto_aim->pitch, auto_aim->is_fire, auto_aim->robot_id, auto_aim->wr, auto_aim->distance);
}

/**
 * @brief Encode a auto_aim struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auto_aim C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_aim_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_auto_aim_t* auto_aim)
{
    return mavlink_msg_auto_aim_pack_chan(system_id, component_id, chan, msg, auto_aim->is_detect, auto_aim->yaw, auto_aim->pitch, auto_aim->is_fire, auto_aim->robot_id, auto_aim->wr, auto_aim->distance);
}

/**
 * @brief Encode a auto_aim struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param auto_aim C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auto_aim_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_auto_aim_t* auto_aim)
{
    return mavlink_msg_auto_aim_pack_status(system_id, component_id, _status, msg,  auto_aim->is_detect, auto_aim->yaw, auto_aim->pitch, auto_aim->is_fire, auto_aim->robot_id, auto_aim->wr, auto_aim->distance);
}

/**
 * @brief Send a auto_aim message
 * @param chan MAVLink channel to send the message
 *
 * @param is_detect  is_detect
 * @param yaw [deg] yaw(left is positive)
 * @param pitch [deg] pitch(up is positive)
 * @param is_fire  is_fire
 * @param robot_id  robot_id(1-7, see robomaster manual)
 * @param wr [rpm] rpm
 * @param distance [m] distance
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_auto_aim_send(mavlink_channel_t chan, uint8_t is_detect, float yaw, float pitch, uint8_t is_fire, uint8_t robot_id, float wr, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_auto_aim_LEN];
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, wr);
    _mav_put_float(buf, 12, distance);
    _mav_put_uint8_t(buf, 16, is_detect);
    _mav_put_uint8_t(buf, 17, is_fire);
    _mav_put_uint8_t(buf, 18, robot_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_aim, buf, MAVLINK_MSG_ID_auto_aim_MIN_LEN, MAVLINK_MSG_ID_auto_aim_LEN, MAVLINK_MSG_ID_auto_aim_CRC);
#else
    mavlink_auto_aim_t packet;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.wr = wr;
    packet.distance = distance;
    packet.is_detect = is_detect;
    packet.is_fire = is_fire;
    packet.robot_id = robot_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_aim, (const char *)&packet, MAVLINK_MSG_ID_auto_aim_MIN_LEN, MAVLINK_MSG_ID_auto_aim_LEN, MAVLINK_MSG_ID_auto_aim_CRC);
#endif
}

/**
 * @brief Send a auto_aim message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_auto_aim_send_struct(mavlink_channel_t chan, const mavlink_auto_aim_t* auto_aim)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_auto_aim_send(chan, auto_aim->is_detect, auto_aim->yaw, auto_aim->pitch, auto_aim->is_fire, auto_aim->robot_id, auto_aim->wr, auto_aim->distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_aim, (const char *)auto_aim, MAVLINK_MSG_ID_auto_aim_MIN_LEN, MAVLINK_MSG_ID_auto_aim_LEN, MAVLINK_MSG_ID_auto_aim_CRC);
#endif
}

#if MAVLINK_MSG_ID_auto_aim_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_auto_aim_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t is_detect, float yaw, float pitch, uint8_t is_fire, uint8_t robot_id, float wr, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, yaw);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, wr);
    _mav_put_float(buf, 12, distance);
    _mav_put_uint8_t(buf, 16, is_detect);
    _mav_put_uint8_t(buf, 17, is_fire);
    _mav_put_uint8_t(buf, 18, robot_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_aim, buf, MAVLINK_MSG_ID_auto_aim_MIN_LEN, MAVLINK_MSG_ID_auto_aim_LEN, MAVLINK_MSG_ID_auto_aim_CRC);
#else
    mavlink_auto_aim_t *packet = (mavlink_auto_aim_t *)msgbuf;
    packet->yaw = yaw;
    packet->pitch = pitch;
    packet->wr = wr;
    packet->distance = distance;
    packet->is_detect = is_detect;
    packet->is_fire = is_fire;
    packet->robot_id = robot_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_auto_aim, (const char *)packet, MAVLINK_MSG_ID_auto_aim_MIN_LEN, MAVLINK_MSG_ID_auto_aim_LEN, MAVLINK_MSG_ID_auto_aim_CRC);
#endif
}
#endif

#endif

// MESSAGE auto_aim UNPACKING


/**
 * @brief Get field is_detect from auto_aim message
 *
 * @return  is_detect
 */
static inline uint8_t mavlink_msg_auto_aim_get_is_detect(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field yaw from auto_aim message
 *
 * @return [deg] yaw(left is positive)
 */
static inline float mavlink_msg_auto_aim_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from auto_aim message
 *
 * @return [deg] pitch(up is positive)
 */
static inline float mavlink_msg_auto_aim_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field is_fire from auto_aim message
 *
 * @return  is_fire
 */
static inline uint8_t mavlink_msg_auto_aim_get_is_fire(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field robot_id from auto_aim message
 *
 * @return  robot_id(1-7, see robomaster manual)
 */
static inline uint8_t mavlink_msg_auto_aim_get_robot_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field wr from auto_aim message
 *
 * @return [rpm] rpm
 */
static inline float mavlink_msg_auto_aim_get_wr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field distance from auto_aim message
 *
 * @return [m] distance
 */
static inline float mavlink_msg_auto_aim_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a auto_aim message into a struct
 *
 * @param msg The message to decode
 * @param auto_aim C-struct to decode the message contents into
 */
static inline void mavlink_msg_auto_aim_decode(const mavlink_message_t* msg, mavlink_auto_aim_t* auto_aim)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    auto_aim->yaw = mavlink_msg_auto_aim_get_yaw(msg);
    auto_aim->pitch = mavlink_msg_auto_aim_get_pitch(msg);
    auto_aim->wr = mavlink_msg_auto_aim_get_wr(msg);
    auto_aim->distance = mavlink_msg_auto_aim_get_distance(msg);
    auto_aim->is_detect = mavlink_msg_auto_aim_get_is_detect(msg);
    auto_aim->is_fire = mavlink_msg_auto_aim_get_is_fire(msg);
    auto_aim->robot_id = mavlink_msg_auto_aim_get_robot_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_auto_aim_LEN? msg->len : MAVLINK_MSG_ID_auto_aim_LEN;
        memset(auto_aim, 0, MAVLINK_MSG_ID_auto_aim_LEN);
    memcpy(auto_aim, _MAV_PAYLOAD(msg), len);
#endif
}
