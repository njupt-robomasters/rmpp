/** @file
 *    @brief MAVLink comm protocol testsuite generated from msg.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef MSG_TESTSUITE_H
#define MSG_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_msg(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_msg(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_imu >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_imu_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.yaw = packet_in.yaw;
        packet1.pitch = packet_in.pitch;
        packet1.roll = packet_in.roll;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_imu_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_imu_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_pack(system_id, component_id, &msg , packet1.yaw , packet1.pitch , packet1.roll );
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.yaw , packet1.pitch , packet1.roll );
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_send(MAVLINK_COMM_1 , packet1.yaw , packet1.pitch , packet1.roll );
    mavlink_msg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("imu") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_imu) != NULL);
#endif
}

static void mavlink_test_referee(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_referee >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_referee_t packet_in = {
        17.0,17
    };
    mavlink_referee_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.bullet_speed = packet_in.bullet_speed;
        packet1.is_red = packet_in.is_red;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_referee_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_referee_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_referee_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_referee_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_referee_pack(system_id, component_id, &msg , packet1.is_red , packet1.bullet_speed );
    mavlink_msg_referee_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_referee_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.is_red , packet1.bullet_speed );
    mavlink_msg_referee_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_referee_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_referee_send(MAVLINK_COMM_1 , packet1.is_red , packet1.bullet_speed );
    mavlink_msg_referee_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("referee") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_referee) != NULL);
#endif
}

static void mavlink_test_aim(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_aim >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_aim_t packet_in = {
        17.0,45.0,29,96
    };
    mavlink_aim_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.yaw = packet_in.yaw;
        packet1.pitch = packet_in.pitch;
        packet1.is_detected = packet_in.is_detected;
        packet1.is_fire_advise = packet_in.is_fire_advise;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_aim_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_aim_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aim_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_aim_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aim_pack(system_id, component_id, &msg , packet1.is_detected , packet1.is_fire_advise , packet1.yaw , packet1.pitch );
    mavlink_msg_aim_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aim_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.is_detected , packet1.is_fire_advise , packet1.yaw , packet1.pitch );
    mavlink_msg_aim_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_aim_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aim_send(MAVLINK_COMM_1 , packet1.is_detected , packet1.is_fire_advise , packet1.yaw , packet1.pitch );
    mavlink_msg_aim_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("aim") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_aim) != NULL);
#endif
}

static void mavlink_test_ui(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ui >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ui_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,137
    };
    mavlink_ui_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.speed = packet_in.speed;
        packet1.x0 = packet_in.x0;
        packet1.y0 = packet_in.y0;
        packet1.x1 = packet_in.x1;
        packet1.y1 = packet_in.y1;
        packet1.x2 = packet_in.x2;
        packet1.y2 = packet_in.y2;
        packet1.x3 = packet_in.x3;
        packet1.y3 = packet_in.y3;
        packet1.x4 = packet_in.x4;
        packet1.y4 = packet_in.y4;
        packet1.num = packet_in.num;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ui_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ui_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ui_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ui_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ui_pack(system_id, component_id, &msg , packet1.num , packet1.speed , packet1.x0 , packet1.y0 , packet1.x1 , packet1.y1 , packet1.x2 , packet1.y2 , packet1.x3 , packet1.y3 , packet1.x4 , packet1.y4 );
    mavlink_msg_ui_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ui_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.num , packet1.speed , packet1.x0 , packet1.y0 , packet1.x1 , packet1.y1 , packet1.x2 , packet1.y2 , packet1.x3 , packet1.y3 , packet1.x4 , packet1.y4 );
    mavlink_msg_ui_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ui_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ui_send(MAVLINK_COMM_1 , packet1.num , packet1.speed , packet1.x0 , packet1.y0 , packet1.x1 , packet1.y1 , packet1.x2 , packet1.y2 , packet1.x3 , packet1.y3 , packet1.x4 , packet1.y4 );
    mavlink_msg_ui_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ui") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ui) != NULL);
#endif
}

static void mavlink_test_msg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_imu(system_id, component_id, last_msg);
    mavlink_test_referee(system_id, component_id, last_msg);
    mavlink_test_aim(system_id, component_id, last_msg);
    mavlink_test_ui(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MSG_TESTSUITE_H
