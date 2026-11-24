#include "bsp/irq_lock.hpp"
#include "minipc/aim.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

static constexpr std::array<uint8_t, 18> RX_GOLDEN{
    0xA5,
    0x5B,
    0x01,
    0x00,
    0x00,
    0xF0,
    0x41,
    0x00,
    0x00,
    0x70,
    0x41,
    0x01,
    0x00,
    0x00,
    0x60,
    0x40,
    0x91,
    0x34,
};
static constexpr std::array<uint8_t, 21> TX_GOLDEN{
    0xA5,
    0x5A,
    0x00,
    0x00,
    0x80,
    0x3F,
    0x00,
    0x00,
    0x80,
    0xBF,
    0x00,
    0x00,
    0x80,
    0xBF,
    0x00,
    0x00,
    0x00,
    0xC8,
    0x41,
    0x5D,
    0x3D,
};

static Cdc::RxCallback receive;
static std::vector<uint8_t> tx;
static int tx_count = 0;
static float now = 0;

void Cdc::add_rx_callback(RxCallback callback) const {
    assert(id == 2);
    receive = std::move(callback);
}

void Cdc::send(const uint8_t* data, const size_t size) const {
    assert(id == 2);
    tx.assign(data, data + size);
    tx_count++;
}

IrqLock::IrqLock() = default;
IrqLock::~IrqLock() = default;

float Timer::elapsed() const {
    return now - static_cast<float>(last_cycles) * ms;
}

float Timer::restart() {
    dt = elapsed();
    last_cycles = static_cast<uint64_t>(std::lround(now / ms));
    return dt;
}

static void set_rx_crc(std::array<uint8_t, RX_GOLDEN.size()>& frame) {
    constexpr uint16_t TABLE[]{0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
                               0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400};
    uint16_t crc = 0xFFFF;
    for (size_t offset = 2; offset < frame.size() - 2; offset++) {
        crc ^= frame[offset];
        crc = (crc >> 4) ^ TABLE[crc & 15];
        crc = (crc >> 4) ^ TABLE[crc & 15];
    }
    frame[16] = static_cast<uint8_t>(crc);
    frame[17] = static_cast<uint8_t>(crc >> 8);
}

static void check_target(const Aim& aim) {
    assert(aim.is_connect && aim.is_detect && aim.is_fire);
    assert(aim.yaw == 30 * deg && aim.pitch == -15 * deg && aim.distance == 3.5f);
}

static void check_disconnect(const Aim& aim) {
    assert(!aim.is_connect && !aim.is_detect && !aim.is_fire);
    assert(aim.yaw == 0 && aim.pitch == 0 && aim.distance == 0);
}

int main() {
    // 任意位置拆分：半帧不发布，最后一字节到达时回调直接发布
    const Aim::Config config{.cdc = Cdc(2)};
    for (size_t split = 1; split < RX_GOLDEN.size(); split++) {
        Aim aim(config);
        receive(RX_GOLDEN.data(), split);
        aim.update();
        check_disconnect(aim);
        receive(RX_GOLDEN.data() + split, RX_GOLDEN.size() - split);
        check_target(aim);
    }

    // 噪声、重叠帧头后粘包：字段按载荷原样映射，识别标志为 0 时其余字段照常发布
    Aim aim(config);
    auto no_target = RX_GOLDEN;
    no_target[2] = 0;
    set_rx_crc(no_target);
    std::vector<uint8_t> stream{0x33, 0xA5, 0xA5};
    stream.insert(stream.end(), RX_GOLDEN.begin(), RX_GOLDEN.end());
    stream.insert(stream.end(), no_target.begin(), no_target.end());
    receive(stream.data(), stream.size());
    assert(aim.is_connect && !aim.is_detect && aim.is_fire && aim.yaw == 30 * deg);

    // CRC 错误帧不刷新连接，超时后断联复位
    now = 201 * ms;
    for (const size_t offset : {2, 11, 15, 16, 17}) {
        auto bad = RX_GOLDEN;
        bad[offset] ^= 0x80;
        receive(bad.data(), bad.size());
        aim.update();
        check_disconnect(aim);
    }

    // 校验通过即原样接收，不检查浮点有限性
    auto nonfinite = RX_GOLDEN;
    nonfinite[5] = 0x80;
    nonfinite[6] = 0x7F;
    set_rx_crc(nonfinite);
    receive(nonfinite.data(), nonfinite.size());
    assert(aim.is_connect && std::isinf(static_cast<float>(aim.yaw)));

    // 每个位置缺 1 字节后紧跟完整帧，均恢复
    for (size_t offset = 0; offset < RX_GOLDEN.size(); offset++) {
        receive(no_target.data(), no_target.size());
        assert(aim.is_connect && !aim.is_detect);
        std::vector<uint8_t> broken(RX_GOLDEN.begin(), RX_GOLDEN.end());
        broken.erase(broken.begin() + offset);
        broken.insert(broken.end(), RX_GOLDEN.begin(), RX_GOLDEN.end());
        receive(broken.data(), broken.size());
        check_target(aim);
    }

    // 断联只在 update 检查：超时前的半帧与超时后的尾段拼成完整帧时照常接收
    receive(RX_GOLDEN.data(), 8);
    now += 201 * ms;
    aim.update();
    check_disconnect(aim);
    receive(RX_GOLDEN.data() + 8, RX_GOLDEN.size() - 8);
    check_target(aim);

    // 上行：set_*() 直接写发送帧，send() 补 CRC；不做数值检查
    aim.set_imu(1 * deg, 1 * deg, -1 * deg);
    aim.set_referee(false, 25);
    assert(tx_count == 0);
    aim.send();
    assert(tx_count == 1 && std::ranges::equal(tx, TX_GOLDEN));
    aim.send();
    assert(tx_count == 2 && std::ranges::equal(tx, TX_GOLDEN));
    aim.set_referee(true, -1);
    aim.send();
    assert(tx_count == 3 && tx[14] == 1);
    const uint8_t minus_one[]{0x00, 0x00, 0x80, 0xBF};
    assert(std::equal(tx.begin() + 15, tx.begin() + 19, minus_one));
    aim.set_imu(std::numeric_limits<float>::quiet_NaN(), 0, 0);
    aim.send();
    assert(tx_count == 4);
    std::puts("Aim protocol tests passed");
}
