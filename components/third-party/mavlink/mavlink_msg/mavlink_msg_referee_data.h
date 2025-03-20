#pragma once
// MESSAGE REFEREE_DATA PACKING

#define MAVLINK_MSG_ID_REFEREE_DATA 2


typedef struct __mavlink_referee_data_t {
 float bullet_speed; /*< [m/s] bullet_speed*/
 uint8_t team_is_red; /*<  team color(1 is red,0 is bule)*/
} mavlink_referee_data_t;

#define MAVLINK_MSG_ID_REFEREE_DATA_LEN 5
#define MAVLINK_MSG_ID_REFEREE_DATA_MIN_LEN 5
#define MAVLINK_MSG_ID_2_LEN 5
#define MAVLINK_MSG_ID_2_MIN_LEN 5

#define MAVLINK_MSG_ID_REFEREE_DATA_CRC 195
#define MAVLINK_MSG_ID_2_CRC 195



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_REFEREE_DATA { \
    2, \
    "REFEREE_DATA", \
    2, \
    {  { "team_is_red", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_referee_data_t, team_is_red) }, \
         { "bullet_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_referee_data_t, bullet_speed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_REFEREE_DATA { \
    "REFEREE_DATA", \
    2, \
    {  { "team_is_red", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_referee_data_t, team_is_red) }, \
         { "bullet_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_referee_data_t, bullet_speed) }, \
         } \
}
#endif

/**
 * @brief Pack a referee_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param team_is_red  team color(1 is red,0 is bule)
 * @param bullet_speed [m/s] bullet_speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_referee_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t team_is_red, float bullet_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REFEREE_DATA_LEN];
    _mav_put_float(buf, 0, bullet_speed);
    _mav_put_uint8_t(buf, 4, team_is_red);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REFEREE_DATA_LEN);
#else
    mavlink_referee_data_t packet;
    packet.bullet_speed = bullet_speed;
    packet.team_is_red = team_is_red;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REFEREE_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REFEREE_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REFEREE_DATA_MIN_LEN, MAVLINK_MSG_ID_REFEREE_DATA_LEN, MAVLINK_MSG_ID_REFEREE_DATA_CRC);
}

/**
 * @brief Pack a referee_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param team_is_red  team color(1 is red,0 is bule)
 * @param bullet_speed [m/s] bullet_speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_referee_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t team_is_red, float bullet_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REFEREE_DATA_LEN];
    _mav_put_float(buf, 0, bullet_speed);
    _mav_put_uint8_t(buf, 4, team_is_red);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REFEREE_DATA_LEN);
#else
    mavlink_referee_data_t packet;
    packet.bullet_speed = bullet_speed;
    packet.team_is_red = team_is_red;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REFEREE_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REFEREE_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_REFEREE_DATA_MIN_LEN, MAVLINK_MSG_ID_REFEREE_DATA_LEN, MAVLINK_MSG_ID_REFEREE_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_REFEREE_DATA_MIN_LEN, MAVLINK_MSG_ID_REFEREE_DATA_LEN);
#endif
}

/**
 * @brief Pack a referee_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param team_is_red  team color(1 is red,0 is bule)
 * @param bullet_speed [m/s] bullet_speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_referee_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t team_is_red,float bullet_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REFEREE_DATA_LEN];
    _mav_put_float(buf, 0, bullet_speed);
    _mav_put_uint8_t(buf, 4, team_is_red);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REFEREE_DATA_LEN);
#else
    mavlink_referee_data_t packet;
    packet.bullet_speed = bullet_speed;
    packet.team_is_red = team_is_red;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REFEREE_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REFEREE_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REFEREE_DATA_MIN_LEN, MAVLINK_MSG_ID_REFEREE_DATA_LEN, MAVLINK_MSG_ID_REFEREE_DATA_CRC);
}

/**
 * @brief Encode a referee_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param referee_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_referee_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_referee_data_t* referee_data)
{
    return mavlink_msg_referee_data_pack(system_id, component_id, msg, referee_data->team_is_red, referee_data->bullet_speed);
}

/**
 * @brief Encode a referee_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param referee_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_referee_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_referee_data_t* referee_data)
{
    return mavlink_msg_referee_data_pack_chan(system_id, component_id, chan, msg, referee_data->team_is_red, referee_data->bullet_speed);
}

/**
 * @brief Encode a referee_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param referee_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_referee_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_referee_data_t* referee_data)
{
    return mavlink_msg_referee_data_pack_status(system_id, component_id, _status, msg,  referee_data->team_is_red, referee_data->bullet_speed);
}

/**
 * @brief Send a referee_data message
 * @param chan MAVLink channel to send the message
 *
 * @param team_is_red  team color(1 is red,0 is bule)
 * @param bullet_speed [m/s] bullet_speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_referee_data_send(mavlink_channel_t chan, uint8_t team_is_red, float bullet_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REFEREE_DATA_LEN];
    _mav_put_float(buf, 0, bullet_speed);
    _mav_put_uint8_t(buf, 4, team_is_red);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REFEREE_DATA, buf, MAVLINK_MSG_ID_REFEREE_DATA_MIN_LEN, MAVLINK_MSG_ID_REFEREE_DATA_LEN, MAVLINK_MSG_ID_REFEREE_DATA_CRC);
#else
    mavlink_referee_data_t packet;
    packet.bullet_speed = bullet_speed;
    packet.team_is_red = team_is_red;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REFEREE_DATA, (const char *)&packet, MAVLINK_MSG_ID_REFEREE_DATA_MIN_LEN, MAVLINK_MSG_ID_REFEREE_DATA_LEN, MAVLINK_MSG_ID_REFEREE_DATA_CRC);
#endif
}

/**
 * @brief Send a referee_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_referee_data_send_struct(mavlink_channel_t chan, const mavlink_referee_data_t* referee_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_referee_data_send(chan, referee_data->team_is_red, referee_data->bullet_speed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REFEREE_DATA, (const char *)referee_data, MAVLINK_MSG_ID_REFEREE_DATA_MIN_LEN, MAVLINK_MSG_ID_REFEREE_DATA_LEN, MAVLINK_MSG_ID_REFEREE_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_REFEREE_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_referee_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t team_is_red, float bullet_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, bullet_speed);
    _mav_put_uint8_t(buf, 4, team_is_red);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REFEREE_DATA, buf, MAVLINK_MSG_ID_REFEREE_DATA_MIN_LEN, MAVLINK_MSG_ID_REFEREE_DATA_LEN, MAVLINK_MSG_ID_REFEREE_DATA_CRC);
#else
    mavlink_referee_data_t *packet = (mavlink_referee_data_t *)msgbuf;
    packet->bullet_speed = bullet_speed;
    packet->team_is_red = team_is_red;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REFEREE_DATA, (const char *)packet, MAVLINK_MSG_ID_REFEREE_DATA_MIN_LEN, MAVLINK_MSG_ID_REFEREE_DATA_LEN, MAVLINK_MSG_ID_REFEREE_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE REFEREE_DATA UNPACKING


/**
 * @brief Get field team_is_red from referee_data message
 *
 * @return  team color(1 is red,0 is bule)
 */
static inline uint8_t mavlink_msg_referee_data_get_team_is_red(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field bullet_speed from referee_data message
 *
 * @return [m/s] bullet_speed
 */
static inline float mavlink_msg_referee_data_get_bullet_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a referee_data message into a struct
 *
 * @param msg The message to decode
 * @param referee_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_referee_data_decode(const mavlink_message_t* msg, mavlink_referee_data_t* referee_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    referee_data->bullet_speed = mavlink_msg_referee_data_get_bullet_speed(msg);
    referee_data->team_is_red = mavlink_msg_referee_data_get_team_is_red(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_REFEREE_DATA_LEN? msg->len : MAVLINK_MSG_ID_REFEREE_DATA_LEN;
        memset(referee_data, 0, MAVLINK_MSG_ID_REFEREE_DATA_LEN);
    memcpy(referee_data, _MAV_PAYLOAD(msg), len);
#endif
}
