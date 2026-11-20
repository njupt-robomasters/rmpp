#include "support.hpp"
#include "robot/gimbal/gimbal_fold.hpp"
#include "motor/dm4310.hpp"
#include <limits>

static void protocol_test() {
    const uint8_t check_text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(reference_crc(check_text, 9) == 0x4B37);
    const std::array<uint8_t, 17> golden{0xA5, 0x5A, 0, 0, 0x80, 0x3F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x9A, 0xE6};
    CHECK(make_frame(1, 0, 0, 0) == golden);
    int aim_packets = 0;
    Cdc(1).add_rx_callback([&](const uint8_t*, std::size_t) { aim_packets++; });
    Nav nav(Cdc(2), {});
    nav.update();
    CHECK(!nav.is_connect);
    feed(1, golden.data(), golden.size());
    nav.update();
    CHECK(!nav.is_connect && aim_packets == 1);

    // 首帧可跨主循环逐字节到达；接收回调不会直接改公开指令
    for (std::size_t i = 0; i < golden.size(); i++) {
        feed(2, golden.data() + i, 1);
        CHECK(!nav.is_connect);
        nav.update();
        CHECK(nav.is_connect == (i == golden.size() - 1));
    }
    near(nav.command.vx, 1);
    CHECK(nav.command.action_flag == 0);

    const auto other = make_frame(-1.25f, 2.5f, -0.75f, 1);
    for (std::size_t split = 0; split <= other.size(); split++) {
        feed(2, other.data(), split);
        nav.update();
        feed(2, other.data() + split, other.size() - split);
        nav.update();
        near(nav.command.vx, -1.25f);
        near(nav.command.vy, 2.5f);
        near(nav.command.yaw, -0.75f);
        CHECK(nav.command.action_flag == 1);
    }

    // 连续完整帧、重叠帧头、每个位置插入或丢失一个字节，随后均恢复到黄金帧
    for (int position = 0; position < 17; position++) {
        for (const bool insert : {false, true}) {
            std::vector<uint8_t> stream{0x55, 0xA5, 0xA5};
            std::vector<uint8_t> damaged(other.begin(), other.end());
            if (insert)
                damaged.insert(damaged.begin() + position, 0x33);
            else
                damaged.erase(damaged.begin() + position);
            stream.insert(stream.end(), damaged.begin(), damaged.end());
            stream.insert(stream.end(), golden.begin(), golden.end());
            feed(2, stream.data(), stream.size());
            nav.update();
            near(nav.command.vx, 1);
            near(nav.command.vy, 0);
            CHECK(nav.command.action_flag == 0);
        }
    }
    std::vector<uint8_t> joined;
    for (int i = 0; i < 100; i++) {
        const auto frame = make_frame(static_cast<float>(i), -static_cast<float>(i), i * 0.01f, i % 2);
        joined.insert(joined.end(), frame.begin(), frame.end());
    }
    for (std::size_t offset = 0; offset < joined.size(); offset += 7)
        feed(2, joined.data() + offset, std::min<std::size_t>(7, joined.size() - offset));
    nav.update();
    near(nav.command.vx, 99);
    near(nav.command.vy, -99);
    CHECK(nav.command.action_flag == 1);

    now_us += 60000;
    auto corrupted = other;
    corrupted[5] ^= 0x40;
    feed(2, corrupted.data(), corrupted.size());
    for (const auto& invalid : {make_frame(NAN, 0, 0, 0), make_frame(0, INFINITY, 0, 0),
                                make_frame(0, 0, -INFINITY, 0), make_frame(0, 0, 0, 2)})
        feed(2, invalid.data(), invalid.size());
    nav.update();
    near(nav.command.vx, 99);
    now_us += 41000;
    nav.update();
    CHECK(!nav.is_connect);
    near(nav.command.vx, 0);
    near(nav.command.yaw, 0);

    // 超时后首帧可分片恢复，flag=1 直接生效；重复状态保持一致
    for (int i = 0; i < 5; i++) {
        feed(2, other.data(), 8);
        nav.update();
        feed(2, other.data() + 8, 9);
        nav.update();
        CHECK(nav.is_connect && nav.command.action_flag == 1);
    }

    // 第二实例及通道隔离
    Nav second(Cdc(3), {});
    feed(3, golden.data(), golden.size());
    second.update();
    nav.update();
    near(second.command.vx, 1);
    near(nav.command.vx, -1.25f);
    CHECK(aim_packets == 1);

    // 噪声流与恢复：固定随机种子便于复现
    uint32_t random = 7;
    for (int trial = 0; trial < 2000; trial++) {
        std::vector<uint8_t> noise;
        for (int i = 0; i < trial % 91; i++) {
            random = random * 1664525u + 1013904223u;
            noise.push_back(static_cast<uint8_t>(random >> 24));
        }
        noise.insert(noise.end(), golden.begin(), golden.end());
        feed(2, noise.data(), noise.size());
        nav.update();
        near(nav.command.vx, 1);
        CHECK(nav.command.action_flag == 0);
    }
    cdc_callbacks[1].clear();
    cdc_callbacks[2].clear();
    cdc_callbacks[3].clear();
}

class TestMotor : public Motor {
  public:
    TestMotor() : Motor({.can = Can(1)}, 20, 6, 1) {}
    void feedback(const float value) { can_callback({.speed = 0.1f, .angle = value}); }
};

class TestPitch1 : public DM4310 {
  public:
    TestPitch1() : DM4310({.can = Can(2), .tx_id = 0x10, .rx_id = 0x20}) {}
    bool enabled() const { return is_enable; }
};

static void pitch1_test() {
    const Pid::Config speed{.kp = 2, .ki = 0.4f, .max_i = 1, .max_out = 5};
    const Pid::Config position{.kp = 6, .max_out = 8};
    TestMotor yaw1, yaw2, pitch2;
    TestPitch1 pitch1;
    GimbalFold gimbal({yaw1, yaw2, pitch1, pitch2}, {
        .yaw_speed_pid_config = &speed, .yaw_position_pid_config = &position,
        .pitch_speed_pid_config = &speed, .pitch_position_pid_config = &position,
        .pitch_min = -0.3f, .pitch_max = 0.5f,
        .yaw2 = {&speed, &position, -0.7f, 0.7f},
        .pitch1 = {&speed, &position, -1.6f, 0},
        .pitch1_gravity = {.torque = 2, .phase = 0.5f},
        .pitch2_gravity = {.torque = 1, .phase = -0.2f},
    });
    bool other_axis_output = false;
    for (int i = 0; i < 6000; i++) {
        now_us += 1000;
        gimbal.set_enable(i % 71 > 5);
        CHECK(!pitch1.enabled());
        near(pitch1.current_ref, 0);
        near(pitch1.torque_ref, 0);
        yaw1.feedback(0.2f);
        yaw2.feedback(-0.1f);
        pitch2.feedback(0.15f);
        if (i % 79 > 15) {
            const CanFrame feedback{.id = 0x20, .size = 8, .data = {0x10, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 30, 30}};
            for (const auto& callback : can_callbacks)
                callback(feedback);
        }
        gimbal.set_mode(i % 2 ? GimbalFold::ECD_MODE : GimbalFold::IMU_MODE);
        gimbal.set_angle(0.5f, 0.2f);
        gimbal.set_pitch1_angle(-1.5f);
        // 即便直接注入使能和非零目标，下次机构更新也强制失能并清零
        pitch1.set_enable(true);
        pitch1.set_output(5);
        gimbal.update(0.1f, 0.1f);
        CHECK(!pitch1.enabled());
        near(pitch1.torque_ref, 0);
        near(pitch1.current_ref, 0);
        pitch1.send();
        CHECK(last_can.id == 0x10 && last_can.size == 8 && last_can.data[7] == 0xFD);
        for (int j = 0; j < 7; j++)
            CHECK(last_can.data[j] == 0xFF);
        other_axis_output |= std::fabs(pitch2.current_ref) > 0.01f;
    }
    CHECK(other_axis_output);
    can_callbacks.clear();
}

int main() {
    protocol_test();
    pitch1_test();
    CHECK(irq_depth == 0);
    std::printf("Nav and pitch1: %d checks passed\n", checks);
}
