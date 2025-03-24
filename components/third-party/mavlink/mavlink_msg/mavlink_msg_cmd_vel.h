#pragma once
// MESSAGE CMD_VEL PACKING

#define MAVLINK_MSG_ID_CMD_VEL 7


typedef struct __mavlink_cmd_vel_t {
 float x_vel; /*< [m/s] x_vel*/
 float y_vel; /*< [m/s] y_vel*/
 float z_vel; /*< [m/s] yaw_vel*/
} mavlink_cmd_vel_t;

#define MAVLINK_MSG_ID_CMD_VEL_LEN 12
#define MAVLINK_MSG_ID_CMD_VEL_MIN_LEN 12
#define MAVLINK_MSG_ID_7_LEN 12
#define MAVLINK_MSG_ID_7_MIN_LEN 12

#define MAVLINK_MSG_ID_CMD_VEL_CRC 154
#define MAVLINK_MSG_ID_7_CRC 154



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CMD_VEL { \
    7, \
    "CMD_VEL", \
    3, \
    {  { "x_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_cmd_vel_t, x_vel) }, \
         { "y_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_cmd_vel_t, y_vel) }, \
         { "z_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_cmd_vel_t, z_vel) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CMD_VEL { \
    "CMD_VEL", \
    3, \
    {  { "x_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_cmd_vel_t, x_vel) }, \
         { "y_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_cmd_vel_t, y_vel) }, \
         { "z_vel", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_cmd_vel_t, z_vel) }, \
         } \
}
#endif

/**
 * @brief Pack a cmd_vel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x_vel [m/s] x_vel
 * @param y_vel [m/s] y_vel
 * @param z_vel [m/s] yaw_vel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cmd_vel_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x_vel, float y_vel, float z_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CMD_VEL_LEN];
    _mav_put_float(buf, 0, x_vel);
    _mav_put_float(buf, 4, y_vel);
    _mav_put_float(buf, 8, z_vel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CMD_VEL_LEN);
#else
    mavlink_cmd_vel_t packet;
    packet.x_vel = x_vel;
    packet.y_vel = y_vel;
    packet.z_vel = z_vel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CMD_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CMD_VEL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_CMD_VEL_LEN, MAVLINK_MSG_ID_CMD_VEL_CRC);
}

/**
 * @brief Pack a cmd_vel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param x_vel [m/s] x_vel
 * @param y_vel [m/s] y_vel
 * @param z_vel [m/s] yaw_vel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cmd_vel_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float x_vel, float y_vel, float z_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CMD_VEL_LEN];
    _mav_put_float(buf, 0, x_vel);
    _mav_put_float(buf, 4, y_vel);
    _mav_put_float(buf, 8, z_vel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CMD_VEL_LEN);
#else
    mavlink_cmd_vel_t packet;
    packet.x_vel = x_vel;
    packet.y_vel = y_vel;
    packet.z_vel = z_vel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CMD_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CMD_VEL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_CMD_VEL_LEN, MAVLINK_MSG_ID_CMD_VEL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_CMD_VEL_LEN);
#endif
}

/**
 * @brief Pack a cmd_vel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x_vel [m/s] x_vel
 * @param y_vel [m/s] y_vel
 * @param z_vel [m/s] yaw_vel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cmd_vel_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x_vel,float y_vel,float z_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CMD_VEL_LEN];
    _mav_put_float(buf, 0, x_vel);
    _mav_put_float(buf, 4, y_vel);
    _mav_put_float(buf, 8, z_vel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CMD_VEL_LEN);
#else
    mavlink_cmd_vel_t packet;
    packet.x_vel = x_vel;
    packet.y_vel = y_vel;
    packet.z_vel = z_vel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CMD_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CMD_VEL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_CMD_VEL_LEN, MAVLINK_MSG_ID_CMD_VEL_CRC);
}

/**
 * @brief Encode a cmd_vel struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cmd_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cmd_vel_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cmd_vel_t* cmd_vel)
{
    return mavlink_msg_cmd_vel_pack(system_id, component_id, msg, cmd_vel->x_vel, cmd_vel->y_vel, cmd_vel->z_vel);
}

/**
 * @brief Encode a cmd_vel struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cmd_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cmd_vel_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cmd_vel_t* cmd_vel)
{
    return mavlink_msg_cmd_vel_pack_chan(system_id, component_id, chan, msg, cmd_vel->x_vel, cmd_vel->y_vel, cmd_vel->z_vel);
}

/**
 * @brief Encode a cmd_vel struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param cmd_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cmd_vel_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_cmd_vel_t* cmd_vel)
{
    return mavlink_msg_cmd_vel_pack_status(system_id, component_id, _status, msg,  cmd_vel->x_vel, cmd_vel->y_vel, cmd_vel->z_vel);
}

/**
 * @brief Send a cmd_vel message
 * @param chan MAVLink channel to send the message
 *
 * @param x_vel [m/s] x_vel
 * @param y_vel [m/s] y_vel
 * @param z_vel [m/s] yaw_vel
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cmd_vel_send(mavlink_channel_t chan, float x_vel, float y_vel, float z_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CMD_VEL_LEN];
    _mav_put_float(buf, 0, x_vel);
    _mav_put_float(buf, 4, y_vel);
    _mav_put_float(buf, 8, z_vel);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_VEL, buf, MAVLINK_MSG_ID_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_CMD_VEL_LEN, MAVLINK_MSG_ID_CMD_VEL_CRC);
#else
    mavlink_cmd_vel_t packet;
    packet.x_vel = x_vel;
    packet.y_vel = y_vel;
    packet.z_vel = z_vel;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_VEL, (const char *)&packet, MAVLINK_MSG_ID_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_CMD_VEL_LEN, MAVLINK_MSG_ID_CMD_VEL_CRC);
#endif
}

/**
 * @brief Send a cmd_vel message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cmd_vel_send_struct(mavlink_channel_t chan, const mavlink_cmd_vel_t* cmd_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cmd_vel_send(chan, cmd_vel->x_vel, cmd_vel->y_vel, cmd_vel->z_vel);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_VEL, (const char *)cmd_vel, MAVLINK_MSG_ID_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_CMD_VEL_LEN, MAVLINK_MSG_ID_CMD_VEL_CRC);
#endif
}

#if MAVLINK_MSG_ID_CMD_VEL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cmd_vel_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x_vel, float y_vel, float z_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x_vel);
    _mav_put_float(buf, 4, y_vel);
    _mav_put_float(buf, 8, z_vel);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_VEL, buf, MAVLINK_MSG_ID_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_CMD_VEL_LEN, MAVLINK_MSG_ID_CMD_VEL_CRC);
#else
    mavlink_cmd_vel_t *packet = (mavlink_cmd_vel_t *)msgbuf;
    packet->x_vel = x_vel;
    packet->y_vel = y_vel;
    packet->z_vel = z_vel;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CMD_VEL, (const char *)packet, MAVLINK_MSG_ID_CMD_VEL_MIN_LEN, MAVLINK_MSG_ID_CMD_VEL_LEN, MAVLINK_MSG_ID_CMD_VEL_CRC);
#endif
}
#endif

#endif

// MESSAGE CMD_VEL UNPACKING


/**
 * @brief Get field x_vel from cmd_vel message
 *
 * @return [m/s] x_vel
 */
static inline float mavlink_msg_cmd_vel_get_x_vel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y_vel from cmd_vel message
 *
 * @return [m/s] y_vel
 */
static inline float mavlink_msg_cmd_vel_get_y_vel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z_vel from cmd_vel message
 *
 * @return [m/s] yaw_vel
 */
static inline float mavlink_msg_cmd_vel_get_z_vel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a cmd_vel message into a struct
 *
 * @param msg The message to decode
 * @param cmd_vel C-struct to decode the message contents into
 */
static inline void mavlink_msg_cmd_vel_decode(const mavlink_message_t* msg, mavlink_cmd_vel_t* cmd_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cmd_vel->x_vel = mavlink_msg_cmd_vel_get_x_vel(msg);
    cmd_vel->y_vel = mavlink_msg_cmd_vel_get_y_vel(msg);
    cmd_vel->z_vel = mavlink_msg_cmd_vel_get_z_vel(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CMD_VEL_LEN? msg->len : MAVLINK_MSG_ID_CMD_VEL_LEN;
        memset(cmd_vel, 0, MAVLINK_MSG_ID_CMD_VEL_LEN);
    memcpy(cmd_vel, _MAV_PAYLOAD(msg), len);
#endif
}
