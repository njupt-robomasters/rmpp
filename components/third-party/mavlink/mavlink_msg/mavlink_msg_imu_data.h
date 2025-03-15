#pragma once
// MESSAGE IMU_DATA PACKING

#define MAVLINK_MSG_ID_IMU_DATA 0


typedef struct __mavlink_imu_data_t {
 float roll; /*< [deg] roll*/
 float pitch; /*< [deg] pitch*/
 float yaw; /*< [deg] yaw*/
} mavlink_imu_data_t;

#define MAVLINK_MSG_ID_IMU_DATA_LEN 12
#define MAVLINK_MSG_ID_IMU_DATA_MIN_LEN 12
#define MAVLINK_MSG_ID_0_LEN 12
#define MAVLINK_MSG_ID_0_MIN_LEN 12

#define MAVLINK_MSG_ID_IMU_DATA_CRC 30
#define MAVLINK_MSG_ID_0_CRC 30



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMU_DATA { \
    0, \
    "IMU_DATA", \
    3, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_imu_data_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_imu_data_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_imu_data_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMU_DATA { \
    "IMU_DATA", \
    3, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_imu_data_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_imu_data_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_imu_data_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a imu_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll [deg] roll
 * @param pitch [deg] pitch
 * @param yaw [deg] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_DATA_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_DATA_LEN);
#else
    mavlink_imu_data_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_IMU_DATA_LEN, MAVLINK_MSG_ID_IMU_DATA_CRC);
}

/**
 * @brief Pack a imu_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll [deg] roll
 * @param pitch [deg] pitch
 * @param yaw [deg] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_DATA_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_DATA_LEN);
#else
    mavlink_imu_data_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_IMU_DATA_LEN, MAVLINK_MSG_ID_IMU_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_IMU_DATA_LEN);
#endif
}

/**
 * @brief Pack a imu_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll [deg] roll
 * @param pitch [deg] pitch
 * @param yaw [deg] yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_DATA_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_DATA_LEN);
#else
    mavlink_imu_data_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_IMU_DATA_LEN, MAVLINK_MSG_ID_IMU_DATA_CRC);
}

/**
 * @brief Encode a imu_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_data_t* imu_data)
{
    return mavlink_msg_imu_data_pack(system_id, component_id, msg, imu_data->roll, imu_data->pitch, imu_data->yaw);
}

/**
 * @brief Encode a imu_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param imu_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_imu_data_t* imu_data)
{
    return mavlink_msg_imu_data_pack_chan(system_id, component_id, chan, msg, imu_data->roll, imu_data->pitch, imu_data->yaw);
}

/**
 * @brief Encode a imu_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param imu_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_imu_data_t* imu_data)
{
    return mavlink_msg_imu_data_pack_status(system_id, component_id, _status, msg,  imu_data->roll, imu_data->pitch, imu_data->yaw);
}

/**
 * @brief Send a imu_data message
 * @param chan MAVLink channel to send the message
 *
 * @param roll [deg] roll
 * @param pitch [deg] pitch
 * @param yaw [deg] yaw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_data_send(mavlink_channel_t chan, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_DATA_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_DATA, buf, MAVLINK_MSG_ID_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_IMU_DATA_LEN, MAVLINK_MSG_ID_IMU_DATA_CRC);
#else
    mavlink_imu_data_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_DATA, (const char *)&packet, MAVLINK_MSG_ID_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_IMU_DATA_LEN, MAVLINK_MSG_ID_IMU_DATA_CRC);
#endif
}

/**
 * @brief Send a imu_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_imu_data_send_struct(mavlink_channel_t chan, const mavlink_imu_data_t* imu_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_imu_data_send(chan, imu_data->roll, imu_data->pitch, imu_data->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_DATA, (const char *)imu_data, MAVLINK_MSG_ID_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_IMU_DATA_LEN, MAVLINK_MSG_ID_IMU_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMU_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_imu_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_DATA, buf, MAVLINK_MSG_ID_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_IMU_DATA_LEN, MAVLINK_MSG_ID_IMU_DATA_CRC);
#else
    mavlink_imu_data_t *packet = (mavlink_imu_data_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_DATA, (const char *)packet, MAVLINK_MSG_ID_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_IMU_DATA_LEN, MAVLINK_MSG_ID_IMU_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE IMU_DATA UNPACKING


/**
 * @brief Get field roll from imu_data message
 *
 * @return [deg] roll
 */
static inline float mavlink_msg_imu_data_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from imu_data message
 *
 * @return [deg] pitch
 */
static inline float mavlink_msg_imu_data_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from imu_data message
 *
 * @return [deg] yaw
 */
static inline float mavlink_msg_imu_data_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a imu_data message into a struct
 *
 * @param msg The message to decode
 * @param imu_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_data_decode(const mavlink_message_t* msg, mavlink_imu_data_t* imu_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    imu_data->roll = mavlink_msg_imu_data_get_roll(msg);
    imu_data->pitch = mavlink_msg_imu_data_get_pitch(msg);
    imu_data->yaw = mavlink_msg_imu_data_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMU_DATA_LEN? msg->len : MAVLINK_MSG_ID_IMU_DATA_LEN;
        memset(imu_data, 0, MAVLINK_MSG_ID_IMU_DATA_LEN);
    memcpy(imu_data, _MAV_PAYLOAD(msg), len);
#endif
}
