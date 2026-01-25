#pragma once
// MESSAGE ui PACKING

#define MAVLINK_MSG_ID_ui 11


typedef struct __mavlink_ui_t {
 float speed; /*< [rpm] rpm*/
 float x0; /*<  x0*/
 float y0; /*<  y0*/
 float x1; /*<  x1*/
 float y1; /*<  y1*/
 float x2; /*<  x2*/
 float y2; /*<  y2*/
 float x3; /*<  x3*/
 float y3; /*<  y3*/
 float x4; /*<  x4*/
 float y4; /*<  y4*/
 uint8_t num; /*<  num*/
} mavlink_ui_t;

#define MAVLINK_MSG_ID_ui_LEN 45
#define MAVLINK_MSG_ID_ui_MIN_LEN 45
#define MAVLINK_MSG_ID_11_LEN 45
#define MAVLINK_MSG_ID_11_MIN_LEN 45

#define MAVLINK_MSG_ID_ui_CRC 84
#define MAVLINK_MSG_ID_11_CRC 84



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ui { \
    11, \
    "ui", \
    12, \
    {  { "num", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_ui_t, num) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ui_t, speed) }, \
         { "x0", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ui_t, x0) }, \
         { "y0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ui_t, y0) }, \
         { "x1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ui_t, x1) }, \
         { "y1", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ui_t, y1) }, \
         { "x2", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ui_t, x2) }, \
         { "y2", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ui_t, y2) }, \
         { "x3", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ui_t, x3) }, \
         { "y3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ui_t, y3) }, \
         { "x4", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ui_t, x4) }, \
         { "y4", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_ui_t, y4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ui { \
    "ui", \
    12, \
    {  { "num", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_ui_t, num) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ui_t, speed) }, \
         { "x0", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ui_t, x0) }, \
         { "y0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ui_t, y0) }, \
         { "x1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ui_t, x1) }, \
         { "y1", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ui_t, y1) }, \
         { "x2", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ui_t, x2) }, \
         { "y2", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ui_t, y2) }, \
         { "x3", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ui_t, x3) }, \
         { "y3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ui_t, y3) }, \
         { "x4", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ui_t, x4) }, \
         { "y4", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_ui_t, y4) }, \
         } \
}
#endif

/**
 * @brief Pack a ui message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param num  num
 * @param speed [rpm] rpm
 * @param x0  x0
 * @param y0  y0
 * @param x1  x1
 * @param y1  y1
 * @param x2  x2
 * @param y2  y2
 * @param x3  x3
 * @param y3  y3
 * @param x4  x4
 * @param y4  y4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ui_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t num, float speed, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ui_LEN];
    _mav_put_float(buf, 0, speed);
    _mav_put_float(buf, 4, x0);
    _mav_put_float(buf, 8, y0);
    _mav_put_float(buf, 12, x1);
    _mav_put_float(buf, 16, y1);
    _mav_put_float(buf, 20, x2);
    _mav_put_float(buf, 24, y2);
    _mav_put_float(buf, 28, x3);
    _mav_put_float(buf, 32, y3);
    _mav_put_float(buf, 36, x4);
    _mav_put_float(buf, 40, y4);
    _mav_put_uint8_t(buf, 44, num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ui_LEN);
#else
    mavlink_ui_t packet;
    packet.speed = speed;
    packet.x0 = x0;
    packet.y0 = y0;
    packet.x1 = x1;
    packet.y1 = y1;
    packet.x2 = x2;
    packet.y2 = y2;
    packet.x3 = x3;
    packet.y3 = y3;
    packet.x4 = x4;
    packet.y4 = y4;
    packet.num = num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ui_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ui;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ui_MIN_LEN, MAVLINK_MSG_ID_ui_LEN, MAVLINK_MSG_ID_ui_CRC);
}

/**
 * @brief Pack a ui message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param num  num
 * @param speed [rpm] rpm
 * @param x0  x0
 * @param y0  y0
 * @param x1  x1
 * @param y1  y1
 * @param x2  x2
 * @param y2  y2
 * @param x3  x3
 * @param y3  y3
 * @param x4  x4
 * @param y4  y4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ui_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t num, float speed, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ui_LEN];
    _mav_put_float(buf, 0, speed);
    _mav_put_float(buf, 4, x0);
    _mav_put_float(buf, 8, y0);
    _mav_put_float(buf, 12, x1);
    _mav_put_float(buf, 16, y1);
    _mav_put_float(buf, 20, x2);
    _mav_put_float(buf, 24, y2);
    _mav_put_float(buf, 28, x3);
    _mav_put_float(buf, 32, y3);
    _mav_put_float(buf, 36, x4);
    _mav_put_float(buf, 40, y4);
    _mav_put_uint8_t(buf, 44, num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ui_LEN);
#else
    mavlink_ui_t packet;
    packet.speed = speed;
    packet.x0 = x0;
    packet.y0 = y0;
    packet.x1 = x1;
    packet.y1 = y1;
    packet.x2 = x2;
    packet.y2 = y2;
    packet.x3 = x3;
    packet.y3 = y3;
    packet.x4 = x4;
    packet.y4 = y4;
    packet.num = num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ui_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ui;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ui_MIN_LEN, MAVLINK_MSG_ID_ui_LEN, MAVLINK_MSG_ID_ui_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ui_MIN_LEN, MAVLINK_MSG_ID_ui_LEN);
#endif
}

/**
 * @brief Pack a ui message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param num  num
 * @param speed [rpm] rpm
 * @param x0  x0
 * @param y0  y0
 * @param x1  x1
 * @param y1  y1
 * @param x2  x2
 * @param y2  y2
 * @param x3  x3
 * @param y3  y3
 * @param x4  x4
 * @param y4  y4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ui_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t num,float speed,float x0,float y0,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ui_LEN];
    _mav_put_float(buf, 0, speed);
    _mav_put_float(buf, 4, x0);
    _mav_put_float(buf, 8, y0);
    _mav_put_float(buf, 12, x1);
    _mav_put_float(buf, 16, y1);
    _mav_put_float(buf, 20, x2);
    _mav_put_float(buf, 24, y2);
    _mav_put_float(buf, 28, x3);
    _mav_put_float(buf, 32, y3);
    _mav_put_float(buf, 36, x4);
    _mav_put_float(buf, 40, y4);
    _mav_put_uint8_t(buf, 44, num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ui_LEN);
#else
    mavlink_ui_t packet;
    packet.speed = speed;
    packet.x0 = x0;
    packet.y0 = y0;
    packet.x1 = x1;
    packet.y1 = y1;
    packet.x2 = x2;
    packet.y2 = y2;
    packet.x3 = x3;
    packet.y3 = y3;
    packet.x4 = x4;
    packet.y4 = y4;
    packet.num = num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ui_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ui;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ui_MIN_LEN, MAVLINK_MSG_ID_ui_LEN, MAVLINK_MSG_ID_ui_CRC);
}

/**
 * @brief Encode a ui struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ui C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ui_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ui_t* ui)
{
    return mavlink_msg_ui_pack(system_id, component_id, msg, ui->num, ui->speed, ui->x0, ui->y0, ui->x1, ui->y1, ui->x2, ui->y2, ui->x3, ui->y3, ui->x4, ui->y4);
}

/**
 * @brief Encode a ui struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ui C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ui_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ui_t* ui)
{
    return mavlink_msg_ui_pack_chan(system_id, component_id, chan, msg, ui->num, ui->speed, ui->x0, ui->y0, ui->x1, ui->y1, ui->x2, ui->y2, ui->x3, ui->y3, ui->x4, ui->y4);
}

/**
 * @brief Encode a ui struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ui C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ui_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ui_t* ui)
{
    return mavlink_msg_ui_pack_status(system_id, component_id, _status, msg,  ui->num, ui->speed, ui->x0, ui->y0, ui->x1, ui->y1, ui->x2, ui->y2, ui->x3, ui->y3, ui->x4, ui->y4);
}

/**
 * @brief Send a ui message
 * @param chan MAVLink channel to send the message
 *
 * @param num  num
 * @param speed [rpm] rpm
 * @param x0  x0
 * @param y0  y0
 * @param x1  x1
 * @param y1  y1
 * @param x2  x2
 * @param y2  y2
 * @param x3  x3
 * @param y3  y3
 * @param x4  x4
 * @param y4  y4
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ui_send(mavlink_channel_t chan, uint8_t num, float speed, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ui_LEN];
    _mav_put_float(buf, 0, speed);
    _mav_put_float(buf, 4, x0);
    _mav_put_float(buf, 8, y0);
    _mav_put_float(buf, 12, x1);
    _mav_put_float(buf, 16, y1);
    _mav_put_float(buf, 20, x2);
    _mav_put_float(buf, 24, y2);
    _mav_put_float(buf, 28, x3);
    _mav_put_float(buf, 32, y3);
    _mav_put_float(buf, 36, x4);
    _mav_put_float(buf, 40, y4);
    _mav_put_uint8_t(buf, 44, num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ui, buf, MAVLINK_MSG_ID_ui_MIN_LEN, MAVLINK_MSG_ID_ui_LEN, MAVLINK_MSG_ID_ui_CRC);
#else
    mavlink_ui_t packet;
    packet.speed = speed;
    packet.x0 = x0;
    packet.y0 = y0;
    packet.x1 = x1;
    packet.y1 = y1;
    packet.x2 = x2;
    packet.y2 = y2;
    packet.x3 = x3;
    packet.y3 = y3;
    packet.x4 = x4;
    packet.y4 = y4;
    packet.num = num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ui, (const char *)&packet, MAVLINK_MSG_ID_ui_MIN_LEN, MAVLINK_MSG_ID_ui_LEN, MAVLINK_MSG_ID_ui_CRC);
#endif
}

/**
 * @brief Send a ui message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ui_send_struct(mavlink_channel_t chan, const mavlink_ui_t* ui)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ui_send(chan, ui->num, ui->speed, ui->x0, ui->y0, ui->x1, ui->y1, ui->x2, ui->y2, ui->x3, ui->y3, ui->x4, ui->y4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ui, (const char *)ui, MAVLINK_MSG_ID_ui_MIN_LEN, MAVLINK_MSG_ID_ui_LEN, MAVLINK_MSG_ID_ui_CRC);
#endif
}

#if MAVLINK_MSG_ID_ui_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ui_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t num, float speed, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, speed);
    _mav_put_float(buf, 4, x0);
    _mav_put_float(buf, 8, y0);
    _mav_put_float(buf, 12, x1);
    _mav_put_float(buf, 16, y1);
    _mav_put_float(buf, 20, x2);
    _mav_put_float(buf, 24, y2);
    _mav_put_float(buf, 28, x3);
    _mav_put_float(buf, 32, y3);
    _mav_put_float(buf, 36, x4);
    _mav_put_float(buf, 40, y4);
    _mav_put_uint8_t(buf, 44, num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ui, buf, MAVLINK_MSG_ID_ui_MIN_LEN, MAVLINK_MSG_ID_ui_LEN, MAVLINK_MSG_ID_ui_CRC);
#else
    mavlink_ui_t *packet = (mavlink_ui_t *)msgbuf;
    packet->speed = speed;
    packet->x0 = x0;
    packet->y0 = y0;
    packet->x1 = x1;
    packet->y1 = y1;
    packet->x2 = x2;
    packet->y2 = y2;
    packet->x3 = x3;
    packet->y3 = y3;
    packet->x4 = x4;
    packet->y4 = y4;
    packet->num = num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ui, (const char *)packet, MAVLINK_MSG_ID_ui_MIN_LEN, MAVLINK_MSG_ID_ui_LEN, MAVLINK_MSG_ID_ui_CRC);
#endif
}
#endif

#endif

// MESSAGE ui UNPACKING


/**
 * @brief Get field num from ui message
 *
 * @return  num
 */
static inline uint8_t mavlink_msg_ui_get_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field speed from ui message
 *
 * @return [rpm] rpm
 */
static inline float mavlink_msg_ui_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field x0 from ui message
 *
 * @return  x0
 */
static inline float mavlink_msg_ui_get_x0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y0 from ui message
 *
 * @return  y0
 */
static inline float mavlink_msg_ui_get_y0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field x1 from ui message
 *
 * @return  x1
 */
static inline float mavlink_msg_ui_get_x1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field y1 from ui message
 *
 * @return  y1
 */
static inline float mavlink_msg_ui_get_y1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field x2 from ui message
 *
 * @return  x2
 */
static inline float mavlink_msg_ui_get_x2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field y2 from ui message
 *
 * @return  y2
 */
static inline float mavlink_msg_ui_get_y2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field x3 from ui message
 *
 * @return  x3
 */
static inline float mavlink_msg_ui_get_x3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field y3 from ui message
 *
 * @return  y3
 */
static inline float mavlink_msg_ui_get_y3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field x4 from ui message
 *
 * @return  x4
 */
static inline float mavlink_msg_ui_get_x4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field y4 from ui message
 *
 * @return  y4
 */
static inline float mavlink_msg_ui_get_y4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a ui message into a struct
 *
 * @param msg The message to decode
 * @param ui C-struct to decode the message contents into
 */
static inline void mavlink_msg_ui_decode(const mavlink_message_t* msg, mavlink_ui_t* ui)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ui->speed = mavlink_msg_ui_get_speed(msg);
    ui->x0 = mavlink_msg_ui_get_x0(msg);
    ui->y0 = mavlink_msg_ui_get_y0(msg);
    ui->x1 = mavlink_msg_ui_get_x1(msg);
    ui->y1 = mavlink_msg_ui_get_y1(msg);
    ui->x2 = mavlink_msg_ui_get_x2(msg);
    ui->y2 = mavlink_msg_ui_get_y2(msg);
    ui->x3 = mavlink_msg_ui_get_x3(msg);
    ui->y3 = mavlink_msg_ui_get_y3(msg);
    ui->x4 = mavlink_msg_ui_get_x4(msg);
    ui->y4 = mavlink_msg_ui_get_y4(msg);
    ui->num = mavlink_msg_ui_get_num(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ui_LEN? msg->len : MAVLINK_MSG_ID_ui_LEN;
        memset(ui, 0, MAVLINK_MSG_ID_ui_LEN);
    memcpy(ui, _MAV_PAYLOAD(msg), len);
#endif
}
