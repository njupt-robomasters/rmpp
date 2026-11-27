#include "../../app/leg/leg_up/app.cpp"
#include "minipc/crc16_modbus.hpp"
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <utility>

static uint64_t now_ms = 0;
static unsigned send_count = 0;
static std::array<uint8_t, 24> last_tx;

static Cdc::RxCallback& cdc_receive() {
    static Cdc::RxCallback receive;
    return receive;
}

void Cdc::add_rx_callback(RxCallback callback) const {
    assert(id == 1 && !cdc_receive());
    cdc_receive() = std::move(callback);
}

void Cdc::send(const uint8_t* data, const size_t size) const {
    assert(id == 1 && size == last_tx.size());
    std::memcpy(last_tx.data(), data, size);
    send_count++;
}

void Can::add_rx_callback(RxCallback) const {}

bool Can::can_send() const {
    return true;
}

void Can::send(const CanFrame&) const {}

void Uart::add_rx_callback(RxCallback) const {}

void Imu::update() {}

Melody::Melody(const Config& config)
    : config(config), dt(0), attack_step(0), release_step(0), hold(0), ramp_samples(0) {}

void Melody::play(const Song&) {}

void Melody::update() {}

float Timer::elapsed() const {
    return static_cast<float>(now_ms - last_cycles) * ms;
}

float Timer::restart() {
    dt = elapsed();
    last_cycles = now_ms;
    return dt;
}

static void receive_target(const bool is_detect, const bool is_fire) {
    std::array<uint8_t, 17> frame{0xA1, is_detect, is_fire, 0, 0, 0xF0, 0x41, 0, 0, 0x70, 0x41, 0, 0, 0x60, 0x40};
    const auto crc = crc16_modbus(frame.data(), frame.size() - 2);
    frame[15] = static_cast<uint8_t>(crc);
    frame[16] = static_cast<uint8_t>(crc >> 8u);
    cdc_receive()(frame.data(), frame.size());
}

static void tick() {
    now_ms++;
    loop();
}

int main() {
    for (Motor* motor : std::initializer_list<Motor*>{&yaw, &pitch, &shoot}) {
        motor->is_connect = true;
        motor->config.timeout = 1.0f;
    }
    referee.robot_status.robot_id = 103;
    referee.robot_status.shooter_barrel_heat_limit = 100;
    referee.shoot_data.initial_speed = 25.0f;
    vt13.rc.mode = VT13::S;
    vt13.rc.yaw = 0.5f;
    vt13.rc.pitch = 0.25f;
    tick();
    for (const bool is_aim : {false, true}) {
        vt13.rc.photo = is_aim;
        for (const bool is_detect : {false, true}) {
            for (const bool is_fire : {false, true}) {
                receive_target(is_detect, is_fire);
                vt13.mouse.left = true;
                const Angle last_yaw = gimbal.yaw.angle_ref;
                const Angle last_pitch = gimbal.pitch.angle_ref;
                tick();
                if (is_aim && is_detect) {
                    assert(gimbal.yaw.angle_ref == 30 * deg && gimbal.pitch.angle_ref == 15 * deg);
                    assert(leg_protocol.wr == 0);
                } else {
                    assert(gimbal.yaw.angle_ref > last_yaw && gimbal.pitch.angle_ref < last_pitch);
                    assert(leg_protocol.wr == vt13.yaw_speed);
                }
                const bool has_feed = !is_aim || (is_detect && is_fire);
                assert(has_feed ? shoot.current_ref > 0 : shoot.current_ref == 0);
                assert(shooter.bullet_speed.ref == vt13.config.bullet_speed);
            }
        }
    }
    assert(send_count == 1);
    assert(last_tx[0] == 0xA2 && last_tx[1] == 0);
    const uint8_t payload[]{0x00, 0x00, 0xC8, 0x41, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x80, 0x3F};
    assert(std::memcmp(last_tx.data() + 2, payload, sizeof(payload)) == 0);
    const auto crc = crc16_modbus(last_tx.data(), 22);
    assert(last_tx[22] == (crc & 0xFFu) && last_tx[23] == (crc >> 8u));

    vt13.mouse.left = false;
    vt13.rc.wheel = 0.25f;
    tick();
    assert(shoot.current_ref == 0 && leg_protocol.wr == vt13.wr);

    vt13.rc.wheel = 0;
    vt13.mouse.left = true;
    now_ms += 100;
    const Angle last_yaw = gimbal.yaw.angle_ref;
    tick();
    assert(gimbal.yaw.angle_ref > last_yaw && leg_protocol.wr == vt13.yaw_speed);
    assert(shoot.current_ref == 0);

    receive_target(true, true);
    tick();
    assert(gimbal.yaw.angle_ref == 30 * deg && shoot.current_ref > 0);

    referee.power_heat_data.shooter_17mm_barrel_heat = 50;
    tick();
    assert(shoot.current_ref == 0);
    std::puts("leg_up MiniPC control tests passed");
}
