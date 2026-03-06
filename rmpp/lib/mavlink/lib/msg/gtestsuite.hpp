/** @file
 *	@brief MAVLink comm testsuite protocol generated from msg.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "msg.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(msg, imu)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::msg::msg::imu packet_in{};
    packet_in.yaw = 17.0;
    packet_in.pitch = 45.0;
    packet_in.roll = 73.0;

    mavlink::msg::msg::imu packet1{};
    mavlink::msg::msg::imu packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.roll, packet2.roll);
}

#ifdef TEST_INTEROP
TEST(msg_interop, imu)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_imu_t packet_c {
         17.0, 45.0, 73.0
    };

    mavlink::msg::msg::imu packet_in{};
    packet_in.yaw = 17.0;
    packet_in.pitch = 45.0;
    packet_in.roll = 73.0;

    mavlink::msg::msg::imu packet2{};

    mavlink_msg_imu_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.roll, packet2.roll);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(msg, referee)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::msg::msg::referee packet_in{};
    packet_in.is_red = 151;
    packet_in.game_progress = 218;
    packet_in.stage_remain_time = 17443;
    packet_in.bullet_speed = 17.0;

    mavlink::msg::msg::referee packet1{};
    mavlink::msg::msg::referee packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.is_red, packet2.is_red);
    EXPECT_EQ(packet1.game_progress, packet2.game_progress);
    EXPECT_EQ(packet1.stage_remain_time, packet2.stage_remain_time);
    EXPECT_EQ(packet1.bullet_speed, packet2.bullet_speed);
}

#ifdef TEST_INTEROP
TEST(msg_interop, referee)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_referee_t packet_c {
         17.0, 17443, 151, 218
    };

    mavlink::msg::msg::referee packet_in{};
    packet_in.is_red = 151;
    packet_in.game_progress = 218;
    packet_in.stage_remain_time = 17443;
    packet_in.bullet_speed = 17.0;

    mavlink::msg::msg::referee packet2{};

    mavlink_msg_referee_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.is_red, packet2.is_red);
    EXPECT_EQ(packet_in.game_progress, packet2.game_progress);
    EXPECT_EQ(packet_in.stage_remain_time, packet2.stage_remain_time);
    EXPECT_EQ(packet_in.bullet_speed, packet2.bullet_speed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(msg, auto_aim)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::msg::msg::auto_aim packet_in{};
    packet_in.is_detect = 53;
    packet_in.yaw = 17.0;
    packet_in.pitch = 45.0;
    packet_in.is_fire = 120;
    packet_in.robot_id = 187;
    packet_in.wr = 73.0;
    packet_in.distance = 101.0;

    mavlink::msg::msg::auto_aim packet1{};
    mavlink::msg::msg::auto_aim packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.is_detect, packet2.is_detect);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.is_fire, packet2.is_fire);
    EXPECT_EQ(packet1.robot_id, packet2.robot_id);
    EXPECT_EQ(packet1.wr, packet2.wr);
    EXPECT_EQ(packet1.distance, packet2.distance);
}

#ifdef TEST_INTEROP
TEST(msg_interop, auto_aim)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auto_aim_t packet_c {
         17.0, 45.0, 73.0, 101.0, 53, 120, 187
    };

    mavlink::msg::msg::auto_aim packet_in{};
    packet_in.is_detect = 53;
    packet_in.yaw = 17.0;
    packet_in.pitch = 45.0;
    packet_in.is_fire = 120;
    packet_in.robot_id = 187;
    packet_in.wr = 73.0;
    packet_in.distance = 101.0;

    mavlink::msg::msg::auto_aim packet2{};

    mavlink_msg_auto_aim_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.is_detect, packet2.is_detect);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.is_fire, packet2.is_fire);
    EXPECT_EQ(packet_in.robot_id, packet2.robot_id);
    EXPECT_EQ(packet_in.wr, packet2.wr);
    EXPECT_EQ(packet_in.distance, packet2.distance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(msg, ui)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::msg::msg::ui packet_in{};
    packet_in.x0 = 17.0;
    packet_in.y0 = 45.0;
    packet_in.x1 = 73.0;
    packet_in.y1 = 101.0;
    packet_in.x2 = 129.0;
    packet_in.y2 = 157.0;
    packet_in.x3 = 185.0;
    packet_in.y3 = 213.0;

    mavlink::msg::msg::ui packet1{};
    mavlink::msg::msg::ui packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.x0, packet2.x0);
    EXPECT_EQ(packet1.y0, packet2.y0);
    EXPECT_EQ(packet1.x1, packet2.x1);
    EXPECT_EQ(packet1.y1, packet2.y1);
    EXPECT_EQ(packet1.x2, packet2.x2);
    EXPECT_EQ(packet1.y2, packet2.y2);
    EXPECT_EQ(packet1.x3, packet2.x3);
    EXPECT_EQ(packet1.y3, packet2.y3);
}

#ifdef TEST_INTEROP
TEST(msg_interop, ui)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_ui_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::msg::msg::ui packet_in{};
    packet_in.x0 = 17.0;
    packet_in.y0 = 45.0;
    packet_in.x1 = 73.0;
    packet_in.y1 = 101.0;
    packet_in.x2 = 129.0;
    packet_in.y2 = 157.0;
    packet_in.x3 = 185.0;
    packet_in.y3 = 213.0;

    mavlink::msg::msg::ui packet2{};

    mavlink_msg_ui_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.x0, packet2.x0);
    EXPECT_EQ(packet_in.y0, packet2.y0);
    EXPECT_EQ(packet_in.x1, packet2.x1);
    EXPECT_EQ(packet_in.y1, packet2.y1);
    EXPECT_EQ(packet_in.x2, packet2.x2);
    EXPECT_EQ(packet_in.y2, packet2.y2);
    EXPECT_EQ(packet_in.x3, packet2.x3);
    EXPECT_EQ(packet_in.y3, packet2.y3);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(msg, target_position)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::msg::msg::target_position packet_in{};
    packet_in.x = 17.0;
    packet_in.y = 45.0;

    mavlink::msg::msg::target_position packet1{};
    mavlink::msg::msg::target_position packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
}

#ifdef TEST_INTEROP
TEST(msg_interop, target_position)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_target_position_t packet_c {
         17.0, 45.0
    };

    mavlink::msg::msg::target_position packet_in{};
    packet_in.x = 17.0;
    packet_in.y = 45.0;

    mavlink::msg::msg::target_position packet2{};

    mavlink_msg_target_position_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(msg, current_position)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::msg::msg::current_position packet_in{};
    packet_in.x = 17.0;
    packet_in.y = 45.0;
    packet_in.yaw = 73.0;

    mavlink::msg::msg::current_position packet1{};
    mavlink::msg::msg::current_position packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
}

#ifdef TEST_INTEROP
TEST(msg_interop, current_position)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_current_position_t packet_c {
         17.0, 45.0, 73.0
    };

    mavlink::msg::msg::current_position packet_in{};
    packet_in.x = 17.0;
    packet_in.y = 45.0;
    packet_in.yaw = 73.0;

    mavlink::msg::msg::current_position packet2{};

    mavlink_msg_current_position_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(msg, chassis_speed)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::msg::msg::chassis_speed packet_in{};
    packet_in.vx = 17.0;
    packet_in.vy = 45.0;

    mavlink::msg::msg::chassis_speed packet1{};
    mavlink::msg::msg::chassis_speed packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
}

#ifdef TEST_INTEROP
TEST(msg_interop, chassis_speed)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_chassis_speed_t packet_c {
         17.0, 45.0
    };

    mavlink::msg::msg::chassis_speed packet_in{};
    packet_in.vx = 17.0;
    packet_in.vy = 45.0;

    mavlink::msg::msg::chassis_speed packet2{};

    mavlink_msg_chassis_speed_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
