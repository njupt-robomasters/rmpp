#pragma once
// MESSAGE current_position PACKING

#define MAVLINK_MSG_ID_current_position 5


typedef struct __mavlink_current_position_t {
 float x; /*< [m] x*/
 float y; /*< [m] y*/
 float yaw; /*< [deg] yaw*/
} mavlink_current_position_t;

#define MAVLINK_MSG_ID_current_position_LEN 12
#define MAVLINK_MSG_ID_current_position_MIN_LEN 12
#define MAVLINK_MSG_ID_5_LEN 12
#define MAVLINK_MSG_ID_5_MIN_LEN 12

#define MAVLINK_MSG_ID_current_position_CRC 178
#define MAVLINK_MSG_ID_5_CRC 178



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_current_position { \
    5, \
    "current_position", \
    3, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_current_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_current_position_t, y) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_current_position_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_current_position { \
    "current_position", \
    3, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_current_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_current_position_t, y) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_current_position_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a current_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x [m] x
 * @param y [m] y
 * @param yaw [deg] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_current_position_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_current_position_LEN);
#else
    mavlink_current_position_t packet;
    packet.x = x;
    packet.y = y;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_current_position_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_current_position;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_current_position_MIN_LEN, MAVLINK_MSG_ID_current_position_LEN, MAVLINK_MSG_ID_current_position_CRC);
}

/**
 * @brief Pack a current_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param x [m] x
 * @param y [m] y
 * @param yaw [deg] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_position_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float x, float y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_current_position_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_current_position_LEN);
#else
    mavlink_current_position_t packet;
    packet.x = x;
    packet.y = y;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_current_position_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_current_position;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_current_position_MIN_LEN, MAVLINK_MSG_ID_current_position_LEN, MAVLINK_MSG_ID_current_position_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_current_position_MIN_LEN, MAVLINK_MSG_ID_current_position_LEN);
#endif
}

/**
 * @brief Pack a current_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x [m] x
 * @param y [m] y
 * @param yaw [deg] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_current_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_current_position_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_current_position_LEN);
#else
    mavlink_current_position_t packet;
    packet.x = x;
    packet.y = y;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_current_position_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_current_position;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_current_position_MIN_LEN, MAVLINK_MSG_ID_current_position_LEN, MAVLINK_MSG_ID_current_position_CRC);
}

/**
 * @brief Encode a current_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param current_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_current_position_t* current_position)
{
    return mavlink_msg_current_position_pack(system_id, component_id, msg, current_position->x, current_position->y, current_position->yaw);
}

/**
 * @brief Encode a current_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param current_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_current_position_t* current_position)
{
    return mavlink_msg_current_position_pack_chan(system_id, component_id, chan, msg, current_position->x, current_position->y, current_position->yaw);
}

/**
 * @brief Encode a current_position struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param current_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_current_position_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_current_position_t* current_position)
{
    return mavlink_msg_current_position_pack_status(system_id, component_id, _status, msg,  current_position->x, current_position->y, current_position->yaw);
}

/**
 * @brief Send a current_position message
 * @param chan MAVLink channel to send the message
 *
 * @param x [m] x
 * @param y [m] y
 * @param yaw [deg] yaw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_current_position_send(mavlink_channel_t chan, float x, float y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_current_position_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_current_position, buf, MAVLINK_MSG_ID_current_position_MIN_LEN, MAVLINK_MSG_ID_current_position_LEN, MAVLINK_MSG_ID_current_position_CRC);
#else
    mavlink_current_position_t packet;
    packet.x = x;
    packet.y = y;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_current_position, (const char *)&packet, MAVLINK_MSG_ID_current_position_MIN_LEN, MAVLINK_MSG_ID_current_position_LEN, MAVLINK_MSG_ID_current_position_CRC);
#endif
}

/**
 * @brief Send a current_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_current_position_send_struct(mavlink_channel_t chan, const mavlink_current_position_t* current_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_current_position_send(chan, current_position->x, current_position->y, current_position->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_current_position, (const char *)current_position, MAVLINK_MSG_ID_current_position_MIN_LEN, MAVLINK_MSG_ID_current_position_LEN, MAVLINK_MSG_ID_current_position_CRC);
#endif
}

#if MAVLINK_MSG_ID_current_position_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_current_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_current_position, buf, MAVLINK_MSG_ID_current_position_MIN_LEN, MAVLINK_MSG_ID_current_position_LEN, MAVLINK_MSG_ID_current_position_CRC);
#else
    mavlink_current_position_t *packet = (mavlink_current_position_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_current_position, (const char *)packet, MAVLINK_MSG_ID_current_position_MIN_LEN, MAVLINK_MSG_ID_current_position_LEN, MAVLINK_MSG_ID_current_position_CRC);
#endif
}
#endif

#endif

// MESSAGE current_position UNPACKING


/**
 * @brief Get field x from current_position message
 *
 * @return [m] x
 */
static inline float mavlink_msg_current_position_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from current_position message
 *
 * @return [m] y
 */
static inline float mavlink_msg_current_position_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from current_position message
 *
 * @return [deg] yaw
 */
static inline float mavlink_msg_current_position_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a current_position message into a struct
 *
 * @param msg The message to decode
 * @param current_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_current_position_decode(const mavlink_message_t* msg, mavlink_current_position_t* current_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    current_position->x = mavlink_msg_current_position_get_x(msg);
    current_position->y = mavlink_msg_current_position_get_y(msg);
    current_position->yaw = mavlink_msg_current_position_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_current_position_LEN? msg->len : MAVLINK_MSG_ID_current_position_LEN;
        memset(current_position, 0, MAVLINK_MSG_ID_current_position_LEN);
    memcpy(current_position, _MAV_PAYLOAD(msg), len);
#endif
}
