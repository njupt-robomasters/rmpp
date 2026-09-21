#pragma once
// MESSAGE referee PACKING

#define MAVLINK_MSG_ID_referee 1


typedef struct __mavlink_referee_t {
 float bullet_speed; /*< [m/s] bullet_speed(referee measure value)*/
 uint16_t stage_remain_time; /*<  stage_remain_time(see robomaster serial manual)*/
 uint8_t is_red; /*<  is_red*/
 uint8_t game_progress; /*<  game_progress(see robomaster serial manual)*/
} mavlink_referee_t;

#define MAVLINK_MSG_ID_referee_LEN 8
#define MAVLINK_MSG_ID_referee_MIN_LEN 8
#define MAVLINK_MSG_ID_1_LEN 8
#define MAVLINK_MSG_ID_1_MIN_LEN 8

#define MAVLINK_MSG_ID_referee_CRC 9
#define MAVLINK_MSG_ID_1_CRC 9



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_referee { \
    1, \
    "referee", \
    4, \
    {  { "is_red", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_referee_t, is_red) }, \
         { "game_progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_referee_t, game_progress) }, \
         { "stage_remain_time", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_referee_t, stage_remain_time) }, \
         { "bullet_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_referee_t, bullet_speed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_referee { \
    "referee", \
    4, \
    {  { "is_red", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_referee_t, is_red) }, \
         { "game_progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_referee_t, game_progress) }, \
         { "stage_remain_time", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_referee_t, stage_remain_time) }, \
         { "bullet_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_referee_t, bullet_speed) }, \
         } \
}
#endif

/**
 * @brief Pack a referee message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param is_red  is_red
 * @param game_progress  game_progress(see robomaster serial manual)
 * @param stage_remain_time  stage_remain_time(see robomaster serial manual)
 * @param bullet_speed [m/s] bullet_speed(referee measure value)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_referee_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t is_red, uint8_t game_progress, uint16_t stage_remain_time, float bullet_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_referee_LEN];
    _mav_put_float(buf, 0, bullet_speed);
    _mav_put_uint16_t(buf, 4, stage_remain_time);
    _mav_put_uint8_t(buf, 6, is_red);
    _mav_put_uint8_t(buf, 7, game_progress);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_referee_LEN);
#else
    mavlink_referee_t packet;
    packet.bullet_speed = bullet_speed;
    packet.stage_remain_time = stage_remain_time;
    packet.is_red = is_red;
    packet.game_progress = game_progress;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_referee_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_referee;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_referee_MIN_LEN, MAVLINK_MSG_ID_referee_LEN, MAVLINK_MSG_ID_referee_CRC);
}

/**
 * @brief Pack a referee message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param is_red  is_red
 * @param game_progress  game_progress(see robomaster serial manual)
 * @param stage_remain_time  stage_remain_time(see robomaster serial manual)
 * @param bullet_speed [m/s] bullet_speed(referee measure value)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_referee_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t is_red, uint8_t game_progress, uint16_t stage_remain_time, float bullet_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_referee_LEN];
    _mav_put_float(buf, 0, bullet_speed);
    _mav_put_uint16_t(buf, 4, stage_remain_time);
    _mav_put_uint8_t(buf, 6, is_red);
    _mav_put_uint8_t(buf, 7, game_progress);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_referee_LEN);
#else
    mavlink_referee_t packet;
    packet.bullet_speed = bullet_speed;
    packet.stage_remain_time = stage_remain_time;
    packet.is_red = is_red;
    packet.game_progress = game_progress;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_referee_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_referee;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_referee_MIN_LEN, MAVLINK_MSG_ID_referee_LEN, MAVLINK_MSG_ID_referee_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_referee_MIN_LEN, MAVLINK_MSG_ID_referee_LEN);
#endif
}

/**
 * @brief Pack a referee message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param is_red  is_red
 * @param game_progress  game_progress(see robomaster serial manual)
 * @param stage_remain_time  stage_remain_time(see robomaster serial manual)
 * @param bullet_speed [m/s] bullet_speed(referee measure value)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_referee_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t is_red,uint8_t game_progress,uint16_t stage_remain_time,float bullet_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_referee_LEN];
    _mav_put_float(buf, 0, bullet_speed);
    _mav_put_uint16_t(buf, 4, stage_remain_time);
    _mav_put_uint8_t(buf, 6, is_red);
    _mav_put_uint8_t(buf, 7, game_progress);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_referee_LEN);
#else
    mavlink_referee_t packet;
    packet.bullet_speed = bullet_speed;
    packet.stage_remain_time = stage_remain_time;
    packet.is_red = is_red;
    packet.game_progress = game_progress;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_referee_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_referee;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_referee_MIN_LEN, MAVLINK_MSG_ID_referee_LEN, MAVLINK_MSG_ID_referee_CRC);
}

/**
 * @brief Encode a referee struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param referee C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_referee_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_referee_t* referee)
{
    return mavlink_msg_referee_pack(system_id, component_id, msg, referee->is_red, referee->game_progress, referee->stage_remain_time, referee->bullet_speed);
}

/**
 * @brief Encode a referee struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param referee C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_referee_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_referee_t* referee)
{
    return mavlink_msg_referee_pack_chan(system_id, component_id, chan, msg, referee->is_red, referee->game_progress, referee->stage_remain_time, referee->bullet_speed);
}

/**
 * @brief Encode a referee struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param referee C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_referee_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_referee_t* referee)
{
    return mavlink_msg_referee_pack_status(system_id, component_id, _status, msg,  referee->is_red, referee->game_progress, referee->stage_remain_time, referee->bullet_speed);
}

/**
 * @brief Send a referee message
 * @param chan MAVLink channel to send the message
 *
 * @param is_red  is_red
 * @param game_progress  game_progress(see robomaster serial manual)
 * @param stage_remain_time  stage_remain_time(see robomaster serial manual)
 * @param bullet_speed [m/s] bullet_speed(referee measure value)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_referee_send(mavlink_channel_t chan, uint8_t is_red, uint8_t game_progress, uint16_t stage_remain_time, float bullet_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_referee_LEN];
    _mav_put_float(buf, 0, bullet_speed);
    _mav_put_uint16_t(buf, 4, stage_remain_time);
    _mav_put_uint8_t(buf, 6, is_red);
    _mav_put_uint8_t(buf, 7, game_progress);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_referee, buf, MAVLINK_MSG_ID_referee_MIN_LEN, MAVLINK_MSG_ID_referee_LEN, MAVLINK_MSG_ID_referee_CRC);
#else
    mavlink_referee_t packet;
    packet.bullet_speed = bullet_speed;
    packet.stage_remain_time = stage_remain_time;
    packet.is_red = is_red;
    packet.game_progress = game_progress;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_referee, (const char *)&packet, MAVLINK_MSG_ID_referee_MIN_LEN, MAVLINK_MSG_ID_referee_LEN, MAVLINK_MSG_ID_referee_CRC);
#endif
}

/**
 * @brief Send a referee message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_referee_send_struct(mavlink_channel_t chan, const mavlink_referee_t* referee)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_referee_send(chan, referee->is_red, referee->game_progress, referee->stage_remain_time, referee->bullet_speed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_referee, (const char *)referee, MAVLINK_MSG_ID_referee_MIN_LEN, MAVLINK_MSG_ID_referee_LEN, MAVLINK_MSG_ID_referee_CRC);
#endif
}

#if MAVLINK_MSG_ID_referee_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_referee_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t is_red, uint8_t game_progress, uint16_t stage_remain_time, float bullet_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, bullet_speed);
    _mav_put_uint16_t(buf, 4, stage_remain_time);
    _mav_put_uint8_t(buf, 6, is_red);
    _mav_put_uint8_t(buf, 7, game_progress);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_referee, buf, MAVLINK_MSG_ID_referee_MIN_LEN, MAVLINK_MSG_ID_referee_LEN, MAVLINK_MSG_ID_referee_CRC);
#else
    mavlink_referee_t *packet = (mavlink_referee_t *)msgbuf;
    packet->bullet_speed = bullet_speed;
    packet->stage_remain_time = stage_remain_time;
    packet->is_red = is_red;
    packet->game_progress = game_progress;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_referee, (const char *)packet, MAVLINK_MSG_ID_referee_MIN_LEN, MAVLINK_MSG_ID_referee_LEN, MAVLINK_MSG_ID_referee_CRC);
#endif
}
#endif

#endif

// MESSAGE referee UNPACKING


/**
 * @brief Get field is_red from referee message
 *
 * @return  is_red
 */
static inline uint8_t mavlink_msg_referee_get_is_red(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field game_progress from referee message
 *
 * @return  game_progress(see robomaster serial manual)
 */
static inline uint8_t mavlink_msg_referee_get_game_progress(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field stage_remain_time from referee message
 *
 * @return  stage_remain_time(see robomaster serial manual)
 */
static inline uint16_t mavlink_msg_referee_get_stage_remain_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field bullet_speed from referee message
 *
 * @return [m/s] bullet_speed(referee measure value)
 */
static inline float mavlink_msg_referee_get_bullet_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a referee message into a struct
 *
 * @param msg The message to decode
 * @param referee C-struct to decode the message contents into
 */
static inline void mavlink_msg_referee_decode(const mavlink_message_t* msg, mavlink_referee_t* referee)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    referee->bullet_speed = mavlink_msg_referee_get_bullet_speed(msg);
    referee->stage_remain_time = mavlink_msg_referee_get_stage_remain_time(msg);
    referee->is_red = mavlink_msg_referee_get_is_red(msg);
    referee->game_progress = mavlink_msg_referee_get_game_progress(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_referee_LEN? msg->len : MAVLINK_MSG_ID_referee_LEN;
        memset(referee, 0, MAVLINK_MSG_ID_referee_LEN);
    memcpy(referee, _MAV_PAYLOAD(msg), len);
#endif
}
