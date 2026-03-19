#pragma once
// MESSAGE insta360 PACKING

#define MAVLINK_MSG_ID_insta360 3


typedef struct __mavlink_insta360_t {
 float a0; /*< [deg] a0*/
 float c0; /*<  c0*/
 float a1; /*< [deg] a1*/
 float c1; /*<  c1*/
 float a2; /*< [deg] a2*/
 float c2; /*<  c2*/
} mavlink_insta360_t;

#define MAVLINK_MSG_ID_insta360_LEN 24
#define MAVLINK_MSG_ID_insta360_MIN_LEN 24
#define MAVLINK_MSG_ID_3_LEN 24
#define MAVLINK_MSG_ID_3_MIN_LEN 24

#define MAVLINK_MSG_ID_insta360_CRC 134
#define MAVLINK_MSG_ID_3_CRC 134



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_insta360 { \
    3, \
    "insta360", \
    6, \
    {  { "a0", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_insta360_t, a0) }, \
         { "c0", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_insta360_t, c0) }, \
         { "a1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_insta360_t, a1) }, \
         { "c1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_insta360_t, c1) }, \
         { "a2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_insta360_t, a2) }, \
         { "c2", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_insta360_t, c2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_insta360 { \
    "insta360", \
    6, \
    {  { "a0", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_insta360_t, a0) }, \
         { "c0", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_insta360_t, c0) }, \
         { "a1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_insta360_t, a1) }, \
         { "c1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_insta360_t, c1) }, \
         { "a2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_insta360_t, a2) }, \
         { "c2", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_insta360_t, c2) }, \
         } \
}
#endif

/**
 * @brief Pack a insta360 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param a0 [deg] a0
 * @param c0  c0
 * @param a1 [deg] a1
 * @param c1  c1
 * @param a2 [deg] a2
 * @param c2  c2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_insta360_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float a0, float c0, float a1, float c1, float a2, float c2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_insta360_LEN];
    _mav_put_float(buf, 0, a0);
    _mav_put_float(buf, 4, c0);
    _mav_put_float(buf, 8, a1);
    _mav_put_float(buf, 12, c1);
    _mav_put_float(buf, 16, a2);
    _mav_put_float(buf, 20, c2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_insta360_LEN);
#else
    mavlink_insta360_t packet;
    packet.a0 = a0;
    packet.c0 = c0;
    packet.a1 = a1;
    packet.c1 = c1;
    packet.a2 = a2;
    packet.c2 = c2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_insta360_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_insta360;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_insta360_MIN_LEN, MAVLINK_MSG_ID_insta360_LEN, MAVLINK_MSG_ID_insta360_CRC);
}

/**
 * @brief Pack a insta360 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param a0 [deg] a0
 * @param c0  c0
 * @param a1 [deg] a1
 * @param c1  c1
 * @param a2 [deg] a2
 * @param c2  c2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_insta360_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float a0, float c0, float a1, float c1, float a2, float c2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_insta360_LEN];
    _mav_put_float(buf, 0, a0);
    _mav_put_float(buf, 4, c0);
    _mav_put_float(buf, 8, a1);
    _mav_put_float(buf, 12, c1);
    _mav_put_float(buf, 16, a2);
    _mav_put_float(buf, 20, c2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_insta360_LEN);
#else
    mavlink_insta360_t packet;
    packet.a0 = a0;
    packet.c0 = c0;
    packet.a1 = a1;
    packet.c1 = c1;
    packet.a2 = a2;
    packet.c2 = c2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_insta360_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_insta360;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_insta360_MIN_LEN, MAVLINK_MSG_ID_insta360_LEN, MAVLINK_MSG_ID_insta360_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_insta360_MIN_LEN, MAVLINK_MSG_ID_insta360_LEN);
#endif
}

/**
 * @brief Pack a insta360 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param a0 [deg] a0
 * @param c0  c0
 * @param a1 [deg] a1
 * @param c1  c1
 * @param a2 [deg] a2
 * @param c2  c2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_insta360_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float a0,float c0,float a1,float c1,float a2,float c2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_insta360_LEN];
    _mav_put_float(buf, 0, a0);
    _mav_put_float(buf, 4, c0);
    _mav_put_float(buf, 8, a1);
    _mav_put_float(buf, 12, c1);
    _mav_put_float(buf, 16, a2);
    _mav_put_float(buf, 20, c2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_insta360_LEN);
#else
    mavlink_insta360_t packet;
    packet.a0 = a0;
    packet.c0 = c0;
    packet.a1 = a1;
    packet.c1 = c1;
    packet.a2 = a2;
    packet.c2 = c2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_insta360_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_insta360;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_insta360_MIN_LEN, MAVLINK_MSG_ID_insta360_LEN, MAVLINK_MSG_ID_insta360_CRC);
}

/**
 * @brief Encode a insta360 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param insta360 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_insta360_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_insta360_t* insta360)
{
    return mavlink_msg_insta360_pack(system_id, component_id, msg, insta360->a0, insta360->c0, insta360->a1, insta360->c1, insta360->a2, insta360->c2);
}

/**
 * @brief Encode a insta360 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param insta360 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_insta360_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_insta360_t* insta360)
{
    return mavlink_msg_insta360_pack_chan(system_id, component_id, chan, msg, insta360->a0, insta360->c0, insta360->a1, insta360->c1, insta360->a2, insta360->c2);
}

/**
 * @brief Encode a insta360 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param insta360 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_insta360_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_insta360_t* insta360)
{
    return mavlink_msg_insta360_pack_status(system_id, component_id, _status, msg,  insta360->a0, insta360->c0, insta360->a1, insta360->c1, insta360->a2, insta360->c2);
}

/**
 * @brief Send a insta360 message
 * @param chan MAVLink channel to send the message
 *
 * @param a0 [deg] a0
 * @param c0  c0
 * @param a1 [deg] a1
 * @param c1  c1
 * @param a2 [deg] a2
 * @param c2  c2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_insta360_send(mavlink_channel_t chan, float a0, float c0, float a1, float c1, float a2, float c2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_insta360_LEN];
    _mav_put_float(buf, 0, a0);
    _mav_put_float(buf, 4, c0);
    _mav_put_float(buf, 8, a1);
    _mav_put_float(buf, 12, c1);
    _mav_put_float(buf, 16, a2);
    _mav_put_float(buf, 20, c2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_insta360, buf, MAVLINK_MSG_ID_insta360_MIN_LEN, MAVLINK_MSG_ID_insta360_LEN, MAVLINK_MSG_ID_insta360_CRC);
#else
    mavlink_insta360_t packet;
    packet.a0 = a0;
    packet.c0 = c0;
    packet.a1 = a1;
    packet.c1 = c1;
    packet.a2 = a2;
    packet.c2 = c2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_insta360, (const char *)&packet, MAVLINK_MSG_ID_insta360_MIN_LEN, MAVLINK_MSG_ID_insta360_LEN, MAVLINK_MSG_ID_insta360_CRC);
#endif
}

/**
 * @brief Send a insta360 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_insta360_send_struct(mavlink_channel_t chan, const mavlink_insta360_t* insta360)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_insta360_send(chan, insta360->a0, insta360->c0, insta360->a1, insta360->c1, insta360->a2, insta360->c2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_insta360, (const char *)insta360, MAVLINK_MSG_ID_insta360_MIN_LEN, MAVLINK_MSG_ID_insta360_LEN, MAVLINK_MSG_ID_insta360_CRC);
#endif
}

#if MAVLINK_MSG_ID_insta360_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_insta360_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float a0, float c0, float a1, float c1, float a2, float c2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, a0);
    _mav_put_float(buf, 4, c0);
    _mav_put_float(buf, 8, a1);
    _mav_put_float(buf, 12, c1);
    _mav_put_float(buf, 16, a2);
    _mav_put_float(buf, 20, c2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_insta360, buf, MAVLINK_MSG_ID_insta360_MIN_LEN, MAVLINK_MSG_ID_insta360_LEN, MAVLINK_MSG_ID_insta360_CRC);
#else
    mavlink_insta360_t *packet = (mavlink_insta360_t *)msgbuf;
    packet->a0 = a0;
    packet->c0 = c0;
    packet->a1 = a1;
    packet->c1 = c1;
    packet->a2 = a2;
    packet->c2 = c2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_insta360, (const char *)packet, MAVLINK_MSG_ID_insta360_MIN_LEN, MAVLINK_MSG_ID_insta360_LEN, MAVLINK_MSG_ID_insta360_CRC);
#endif
}
#endif

#endif

// MESSAGE insta360 UNPACKING


/**
 * @brief Get field a0 from insta360 message
 *
 * @return [deg] a0
 */
static inline float mavlink_msg_insta360_get_a0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field c0 from insta360 message
 *
 * @return  c0
 */
static inline float mavlink_msg_insta360_get_c0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field a1 from insta360 message
 *
 * @return [deg] a1
 */
static inline float mavlink_msg_insta360_get_a1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field c1 from insta360 message
 *
 * @return  c1
 */
static inline float mavlink_msg_insta360_get_c1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field a2 from insta360 message
 *
 * @return [deg] a2
 */
static inline float mavlink_msg_insta360_get_a2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field c2 from insta360 message
 *
 * @return  c2
 */
static inline float mavlink_msg_insta360_get_c2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a insta360 message into a struct
 *
 * @param msg The message to decode
 * @param insta360 C-struct to decode the message contents into
 */
static inline void mavlink_msg_insta360_decode(const mavlink_message_t* msg, mavlink_insta360_t* insta360)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    insta360->a0 = mavlink_msg_insta360_get_a0(msg);
    insta360->c0 = mavlink_msg_insta360_get_c0(msg);
    insta360->a1 = mavlink_msg_insta360_get_a1(msg);
    insta360->c1 = mavlink_msg_insta360_get_c1(msg);
    insta360->a2 = mavlink_msg_insta360_get_a2(msg);
    insta360->c2 = mavlink_msg_insta360_get_c2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_insta360_LEN? msg->len : MAVLINK_MSG_ID_insta360_LEN;
        memset(insta360, 0, MAVLINK_MSG_ID_insta360_LEN);
    memcpy(insta360, _MAV_PAYLOAD(msg), len);
#endif
}
