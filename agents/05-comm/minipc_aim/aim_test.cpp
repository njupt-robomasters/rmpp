#include "../15-minipc_nav/support.hpp"
#include "minipc/aim.hpp"
#include <bit>

static std::vector<uint8_t> sent;
static int tx_channel = 0, tx_count = 0;
static bool tx_busy = false;

void Cdc::send(const uint8_t* data, const std::size_t size) const {
    if (tx_busy)
        return;
    sent.assign(data, data + size);
    tx_channel = id;
    tx_count++;
}

static float read_wire_float(const uint8_t* bytes) {
    uint32_t bits = 0;
    for (int i = 0; i < 4; i++)
        bits |= uint32_t(bytes[i]) << (8 * i);
    return std::bit_cast<float>(bits);
}

static void write_wire_float(uint8_t* bytes, const float value) {
    const uint32_t bits = std::bit_cast<uint32_t>(value);
    for (int i = 0; i < 4; i++)
        bytes[i] = static_cast<uint8_t>(bits >> (8 * i));
}

static std::array<uint8_t, 18> frame(const uint8_t detect = 1, const uint8_t fire = 1,
                                     const float yaw = 90, const float pitch = 30, const float distance = 3.5f) {
    std::array<uint8_t, 18> bytes{0xA5, 0x5B, detect};
    write_wire_float(bytes.data() + 3, yaw);
    write_wire_float(bytes.data() + 7, pitch);
    bytes[11] = fire;
    write_wire_float(bytes.data() + 12, distance);
    const uint16_t crc = reference_crc(bytes.data() + 2, 14);
    bytes[16] = static_cast<uint8_t>(crc);
    bytes[17] = static_cast<uint8_t>(crc >> 8);
    return bytes;
}

int main() {
    Aim aim(Cdc(1), {});
    Nav nav({.cdc = Cdc(2)});
    const auto target = frame();
    const auto navigation = make_frame(1, -2, 0.1f, 0, 0, 1);
    aim.set_referee(true, 25);
    aim.update(90 * deg, -30 * deg, -45 * deg);
    CHECK(sent.size() == 21 && tx_channel == 1);
    CHECK(sent[0] == 0xA5 && sent[1] == 0x5A && sent[14] == 1);
    near(read_wire_float(sent.data() + 2), 90);
    near(read_wire_float(sent.data() + 6), 30);
    near(read_wire_float(sent.data() + 10), -45);
    near(read_wire_float(sent.data() + 15), 25);
    CHECK((sent[19] | uint16_t(sent[20]) << 8) == reference_crc(sent.data() + 2, 17));

    // 每毫秒更新，1000 ms 内发送 200 次；每次取最新的头部姿态
    const int initial_count = tx_count;
    for (int i = 0; i < 1000; i++) {
        now_us += 1000;
        aim.update(0.2f, -0.1f, 0.3f);
    }
    CHECK(tx_count - initial_count == 200);
    near(read_wire_float(sent.data() + 2), 0.2f / deg);
    aim.set_referee(false, NAN);
    now_us += 5000;
    aim.update(0, 0, 0);
    CHECK(sent[14] == 0);
    near(read_wire_float(sent.data() + 15), 0);
    const int before_busy = tx_count;
    tx_busy = true;
    now_us += 5000;
    aim.update(0, 0, 0);
    CHECK(tx_count == before_busy);
    tx_busy = false;
    now_us += 5000;
    aim.update(0.4f, 0, 0);
    near(read_wire_float(sent.data() + 2), 0.4f / deg);

    // 首帧分片跨主循环，回调仅写待发布结果
    for (std::size_t i = 0; i < target.size(); i++) {
        feed(1, target.data() + i, 1);
        CHECK(!aim.is_connect_vision);
        aim.update(0, 0, 0);
        CHECK(aim.is_connect_vision == (i == target.size() - 1));
    }
    near(aim.vision.yaw, 90 * deg);
    near(aim.vision.pitch, -30 * deg);
    near(aim.vision.distance, 3.5f);
    CHECK(aim.vision.is_detect && aim.vision.is_fire);

    for (std::size_t split = 0; split <= target.size(); split++) {
        feed(1, target.data(), split);
        aim.update(0, 0, 0);
        feed(1, target.data() + split, target.size() - split);
        aim.update(0, 0, 0);
        CHECK(aim.is_connect_vision && aim.vision.is_fire);
    }

    // 插入、删除、CRC 破坏和重叠 SOF 后均能恢复
    for (int position = 0; position < 18; position++) {
        for (const bool insert : {false, true}) {
            std::vector<uint8_t> damaged(target.begin(), target.end());
            if (insert)
                damaged.insert(damaged.begin() + position, 0x33);
            else
                damaged.erase(damaged.begin() + position);
            std::vector<uint8_t> bytes{0xA5, 0xA5};
            bytes.insert(bytes.end(), damaged.begin(), damaged.end());
            bytes.insert(bytes.end(), target.begin(), target.end());
            feed(1, bytes.data(), bytes.size());
            aim.update(0, 0, 0);
            CHECK(aim.vision.is_fire);
            near(aim.vision.pitch, -30 * deg);
        }
    }

    // 丢目标立即撤销旧角度和开火许可，通信保持在线
    const auto lost = frame(0, 1);
    feed(1, lost.data(), lost.size());
    aim.update(0, 0, 0);
    CHECK(aim.is_connect_vision && !aim.vision.is_detect && !aim.vision.is_fire);
    near(aim.vision.distance, 0);
    near(aim.vision.yaw, 0);
    feed(1, target.data(), target.size());
    aim.update(0, 0, 0);
    now_us += 150000;
    for (const auto& invalid : {frame(2), frame(1, 2), frame(1, 1, NAN), frame(1, 1, 0, INFINITY),
                                frame(1, 1, 0, 0, -1), frame(1, 1, 0, 0, NAN)})
        feed(1, invalid.data(), invalid.size());
    auto bad_crc = target;
    bad_crc[4] ^= 0x80;
    feed(1, bad_crc.data(), bad_crc.size());
    aim.update(0, 0, 0);
    CHECK(aim.is_connect_vision && aim.vision.is_fire);
    now_us += 51000;
    aim.update(0, 0, 0);
    CHECK(!aim.is_connect_vision && !aim.vision.is_detect && !aim.vision.is_fire);
    near(aim.vision.distance, 0);

    // 超时可分片恢复；CDC1/2 隔离，两个协议共用编码实现
    feed(1, target.data(), 9);
    aim.update(0, 0, 0);
    CHECK(!aim.is_connect_vision);
    feed(1, target.data() + 9, 9);
    aim.update(0, 0, 0);
    CHECK(aim.is_connect_vision && aim.vision.is_fire);
    feed(2, navigation.data(), navigation.size());
    nav.update();
    near(nav.vx, 1);
    near(aim.vision.yaw, 90 * deg);

    Aim custom(Cdc(3), {.timeout = 20 * ms});
    feed(3, target.data(), target.size());
    custom.update(0, 0, 0);
    CHECK(custom.is_connect_vision);
    now_us += 21000;
    custom.update(0, 0, 0);
    aim.update(0, 0, 0);
    CHECK(!custom.is_connect_vision && aim.is_connect_vision);

    // 任意噪声及多帧粘包，按最新有效结果发布
    uint32_t random = 19;
    for (int trial = 0; trial < 2000; trial++) {
        std::vector<uint8_t> noise;
        for (int i = 0; i < trial % 73; i++) {
            random = random * 1664525u + 1013904223u;
            noise.push_back(static_cast<uint8_t>(random >> 24));
        }
        noise.insert(noise.end(), target.begin(), target.end());
        noise.insert(noise.end(), lost.begin(), lost.end());
        feed(1, noise.data(), noise.size());
        aim.update(0, 0, 0);
        CHECK(aim.is_connect_vision && !aim.vision.is_fire && !aim.vision.is_detect);
    }
    CHECK(irq_depth == 0);
    std::printf("Aim: %d checks passed\n", checks);
}
