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
    packet_in.is_red = 17;
    packet_in.bullet_speed = 17.0;

    mavlink::msg::msg::referee packet1{};
    mavlink::msg::msg::referee packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.is_red, packet2.is_red);
    EXPECT_EQ(packet1.bullet_speed, packet2.bullet_speed);
}

#ifdef TEST_INTEROP
TEST(msg_interop, referee)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_referee_t packet_c {
         17.0, 17
    };

    mavlink::msg::msg::referee packet_in{};
    packet_in.is_red = 17;
    packet_in.bullet_speed = 17.0;

    mavlink::msg::msg::referee packet2{};

    mavlink_msg_referee_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.is_red, packet2.is_red);
    EXPECT_EQ(packet_in.bullet_speed, packet2.bullet_speed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(msg, aim)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::msg::msg::aim packet_in{};
    packet_in.is_detected = 29;
    packet_in.is_fire_advise = 96;
    packet_in.yaw = 17.0;
    packet_in.pitch = 45.0;

    mavlink::msg::msg::aim packet1{};
    mavlink::msg::msg::aim packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.is_detected, packet2.is_detected);
    EXPECT_EQ(packet1.is_fire_advise, packet2.is_fire_advise);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
}

#ifdef TEST_INTEROP
TEST(msg_interop, aim)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_aim_t packet_c {
         17.0, 45.0, 29, 96
    };

    mavlink::msg::msg::aim packet_in{};
    packet_in.is_detected = 29;
    packet_in.is_fire_advise = 96;
    packet_in.yaw = 17.0;
    packet_in.pitch = 45.0;

    mavlink::msg::msg::aim packet2{};

    mavlink_msg_aim_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.is_detected, packet2.is_detected);
    EXPECT_EQ(packet_in.is_fire_advise, packet2.is_fire_advise);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);

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
    packet_in.num = 137;
    packet_in.speed = 17.0;
    packet_in.x0 = 45.0;
    packet_in.y0 = 73.0;
    packet_in.x1 = 101.0;
    packet_in.y1 = 129.0;
    packet_in.x2 = 157.0;
    packet_in.y2 = 185.0;
    packet_in.x3 = 213.0;
    packet_in.y3 = 241.0;
    packet_in.x4 = 269.0;
    packet_in.y4 = 297.0;

    mavlink::msg::msg::ui packet1{};
    mavlink::msg::msg::ui packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.num, packet2.num);
    EXPECT_EQ(packet1.speed, packet2.speed);
    EXPECT_EQ(packet1.x0, packet2.x0);
    EXPECT_EQ(packet1.y0, packet2.y0);
    EXPECT_EQ(packet1.x1, packet2.x1);
    EXPECT_EQ(packet1.y1, packet2.y1);
    EXPECT_EQ(packet1.x2, packet2.x2);
    EXPECT_EQ(packet1.y2, packet2.y2);
    EXPECT_EQ(packet1.x3, packet2.x3);
    EXPECT_EQ(packet1.y3, packet2.y3);
    EXPECT_EQ(packet1.x4, packet2.x4);
    EXPECT_EQ(packet1.y4, packet2.y4);
}

#ifdef TEST_INTEROP
TEST(msg_interop, ui)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_ui_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 137
    };

    mavlink::msg::msg::ui packet_in{};
    packet_in.num = 137;
    packet_in.speed = 17.0;
    packet_in.x0 = 45.0;
    packet_in.y0 = 73.0;
    packet_in.x1 = 101.0;
    packet_in.y1 = 129.0;
    packet_in.x2 = 157.0;
    packet_in.y2 = 185.0;
    packet_in.x3 = 213.0;
    packet_in.y3 = 241.0;
    packet_in.x4 = 269.0;
    packet_in.y4 = 297.0;

    mavlink::msg::msg::ui packet2{};

    mavlink_msg_ui_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.num, packet2.num);
    EXPECT_EQ(packet_in.speed, packet2.speed);
    EXPECT_EQ(packet_in.x0, packet2.x0);
    EXPECT_EQ(packet_in.y0, packet2.y0);
    EXPECT_EQ(packet_in.x1, packet2.x1);
    EXPECT_EQ(packet_in.y1, packet2.y1);
    EXPECT_EQ(packet_in.x2, packet2.x2);
    EXPECT_EQ(packet_in.y2, packet2.y2);
    EXPECT_EQ(packet_in.x3, packet2.x3);
    EXPECT_EQ(packet_in.y3, packet2.y3);
    EXPECT_EQ(packet_in.x4, packet2.x4);
    EXPECT_EQ(packet_in.y4, packet2.y4);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
