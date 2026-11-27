#include "minipc/crc16_modbus.hpp"
#include "motor/dm4310.hpp"
#include "robot/gimbal_fold.hpp"
#include "support.hpp"
#include <algorithm>
#include <limits>

static void protocol_test() {
    const uint8_t check_text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(reference_crc(check_text, 9) == 0x4B37);
    CHECK(crc16_modbus(check_text, sizeof(check_text)) == 0x4B37u);
    const std::array<uint8_t, 24> golden{0xA3, 0x00, 0, 0, 0x80, 0x3F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x8A, 0x78};
    CHECK(make_frame(1, 0, 0, 0, 0, 0) == golden);
    int aim_packets = 0;
    Cdc(1).add_rx_callback([&](const uint8_t*, std::size_t) { aim_packets++; });
    Nav nav({.cdc = Cdc(2)});
    nav.update();
    CHECK(!nav.is_connect);
    feed(1, golden.data(), golden.size());
    nav.update();
    CHECK(!nav.is_connect && aim_packets == 1);

    // 首帧可跨主循环逐字节到达；最后一字节到达时接收回调直接发布指令
    for (std::size_t i = 0; i < golden.size(); i++) {
        feed(2, golden.data() + i, 1);
        CHECK(nav.is_connect == (i == golden.size() - 1));
        nav.update();
        CHECK(nav.is_connect == (i == golden.size() - 1));
    }
    near(nav.vx, 1);
    near(nav.yaw, 0);
    CHECK(!nav.is_through_hole);

    const auto other = make_frame(-1.25f, 2.5f, -0.75f, 1.5f, -2.5f, 1);
    for (std::size_t split = 0; split <= other.size(); split++) {
        feed(2, other.data(), split);
        nav.update();
        feed(2, other.data() + split, other.size() - split);
        nav.update();
        near(nav.vx, -1.25f);
        near(nav.vy, 2.5f);
        near(nav.yaw, -0.75f);
        near(nav.x, 1.5f);
        near(nav.y, -2.5f);
        CHECK(nav.is_through_hole);
    }

    // 连续完整帧、重叠帧头、每个位置插入或丢失一个字节，随后均恢复到黄金帧
    for (int position = 0; position < 24; position++) {
        for (const bool insert : {false, true}) {
            std::vector<uint8_t> stream{0x55, 0xA3, 0xA3};
            std::vector<uint8_t> damaged(other.begin(), other.end());
            if (insert)
                damaged.insert(damaged.begin() + position, 0x33);
            else
                damaged.erase(damaged.begin() + position);
            stream.insert(stream.end(), damaged.begin(), damaged.end());
            stream.insert(stream.end(), golden.begin(), golden.end());
            feed(2, stream.data(), stream.size());
            nav.update();
            near(nav.vx, 1);
            near(nav.vy, 0);
            near(nav.x, 0);
            near(nav.y, 0);
            CHECK(!nav.is_through_hole);
        }
    }
    std::vector<uint8_t> joined;
    for (int i = 0; i < 100; i++) {
        const auto frame = make_frame(
            static_cast<float>(i), -static_cast<float>(i), i * 0.01f, i * 0.1f, -i * 0.2f, i % 2);
        joined.insert(joined.end(), frame.begin(), frame.end());
    }
    for (std::size_t offset = 0; offset < joined.size(); offset += 7)
        feed(2, joined.data() + offset, std::min<std::size_t>(7, joined.size() - offset));
    nav.update();
    near(nav.vx, 99);
    near(nav.vy, -99);
    near(nav.x, 9.9f);
    near(nav.y, -19.8f);
    CHECK(nav.is_through_hole);

    now_us += 60000;
    auto corrupted = other;
    corrupted[5] ^= 0x40;
    feed(2, corrupted.data(), corrupted.size());
    nav.update();
    near(nav.vx, 99);
    now_us += 41000;
    nav.update();
    CHECK(!nav.is_connect);
    near(nav.vx, 0);
    near(nav.yaw, 0);
    near(nav.x, 0);
    near(nav.y, 0);
    CHECK(!nav.is_through_hole);

    // 超时后首帧可分片恢复，过洞标志直接生效；重复状态保持一致
    for (int i = 0; i < 5; i++) {
        feed(2, other.data(), 8);
        nav.update();
        feed(2, other.data() + 8, other.size() - 8);
        nav.update();
        CHECK(nav.is_connect && nav.is_through_hole);
        near(nav.x, 1.5f);
    }

    // 校验通过即原样接收，不检查浮点有限性
    const auto unchecked = make_frame(NAN, INFINITY, -INFINITY, 3, -4, 2);
    feed(2, unchecked.data(), unchecked.size());
    nav.update();
    CHECK(nav.is_connect && std::isnan(nav.vx) && std::isinf(nav.vy) && nav.is_through_hole);

    // 断联只在 update 检查：超时前的半帧与超时后的尾段拼成完整帧时照常接收
    feed(2, golden.data(), 8);
    now_us += 101000;
    nav.update();
    CHECK(!nav.is_connect);
    feed(2, golden.data() + 8, golden.size() - 8);
    nav.update();
    CHECK(nav.is_connect);
    near(nav.vx, 1);
    feed(2, other.data(), other.size());

    // 第二实例及通道隔离
    Nav second({.cdc = Cdc(3)});
    feed(3, golden.data(), golden.size());
    second.update();
    nav.update();
    near(second.vx, 1);
    near(nav.vx, -1.25f);
    CHECK(aim_packets == 1);

    // 上行编码器角为 π/4 rad，下行转角 +π/2 rad 表示从大 yaw 当前朝向逆时针转 90°
    nav.set_target(1.5f, -2.5f);
    nav.set_yaw_ecd(pi / 4);
    nav.send();
    const std::array<uint8_t, 15> target_golden{0xA4, 0xDB, 0x0F, 0x49, 0x3F, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x20, 0xC0, 0x41, 0x20};
    CHECK(cdc_tx[2].size() == target_golden.size());
    CHECK(std::equal(cdc_tx[2].begin(), cdc_tx[2].end(), target_golden.begin()));
    const std::array<uint8_t, 24> turn_golden{0xA3, 0, 0, 0, 0x80, 0x3F, 0, 0, 0, 0, 0xDB, 0x0F, 0xC9, 0x3F, 0, 0, 0, 0, 0, 0, 0, 0, 0x5D, 0xEB};
    for (int repeat = 0; repeat < 2; repeat++) {
        feed(2, turn_golden.data(), turn_golden.size());
        near(nav.yaw, pi / 2);
    }
    feed(2, golden.data(), golden.size());
    near(nav.yaw, 0);
    nav.set_target(NAN, INFINITY);
    nav.set_yaw_ecd(0);
    nav.send();
    CHECK(cdc_tx[2].size() == target_golden.size());

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
        near(nav.vx, 1);
        CHECK(!nav.is_through_hole);
    }
    cdc_callbacks[1].clear();
    cdc_callbacks[2].clear();
    cdc_callbacks[3].clear();
}

class TestMotor : public Motor {
  public:
    TestMotor() : Motor({.can = Can(1)}, 20, 6, 1) {}

    void feedback(const float value) {
        can_callback({.speed = 0.1f, .angle = value});
    }
};

class TestPitch1 : public DM4310 {
  public:
    TestPitch1() : DM4310({.can = Can(2), .tx_id = 0x10, .rx_id = 0x20}) {}

    bool enabled() const {
        return is_enable;
    }
};

static void pitch1_test() {
    const Pid::Config speed{.kp = 2, .ki = 0.4f, .max_i = 1, .max_out = 5};
    const Pid::Config position{.kp = 6, .max_out = 8};
    TestMotor yaw1, yaw2, pitch2;
    TestPitch1 pitch1;
    GimbalFold gimbal({yaw1, yaw2, pitch1, pitch2}, {
                                                        .yaw_speed_pid_config = &speed,
                                                        .yaw_position_pid_config = &position,
                                                        .pitch_speed_pid_config = &speed,
                                                        .pitch_position_pid_config = &position,
                                                        .pitch_min = -0.3f,
                                                        .pitch_max = 0.5f,
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
