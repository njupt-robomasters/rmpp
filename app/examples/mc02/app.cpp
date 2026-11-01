// mc02 外设自检：电源、RGB、蜂鸣器、IMU 姿态，以及 CAN、串口、CDC 的回环转发
#include "attitude/attitude.hpp"
#include "board/power.hpp"
#include "bsp/can.hpp"
#include "bsp/cdc.hpp"
#include "bsp/imu.hpp"
#include "bsp/rgb.hpp"
#include "bsp/timer.hpp"
#include "bsp/uart.hpp"
#include "misc/melody.hpp"
#include "misc/music/c5d5g5.hpp"
#include <cmath>

static Rgb rgb;
static Melody melody;
static Imu imu;
static Attitude attitude(imu, {});

// 收到的数据原样发回
static void echo() {
    for (const int id : {1, 2, 3})
        Can(id).add_rx_callback([id](const CanFrame& frame) {
            CanFrame reply = frame;
            reply.id = frame.id + 1;
            Can(id).send(reply);
        });
    for (const int id : {1, 7, 10})
        Uart(id).add_rx_callback([id](const uint8_t* data, const std::size_t size) { Uart(id).send(data, size); });
    for (const int id : {1, 2})
        Cdc(id).add_rx_callback([id](const uint8_t* data, const std::size_t size) { Cdc(id).send(data, size); });
}

extern "C" void app_main() {
    set_power(PowerOutput::OUT_5V, true); // CAN 收发器与外设 5 V
    echo();
    melody.play(C5D5G5);

    static constexpr float RGB_PERIOD = 1; // 色彩循环周期，s
    float hue = 0;
    Timer rgb_timer;

    Timer timer;
    while (true) {
        if (timer.elapsed() > 0.001f) {
            timer.restart();
            attitude.update();
            melody.update();

            // WS2812 两次刷新间隔须大于 280 μs，按 10 ms 刷新
            if (rgb_timer.elapsed() > 0.01f) {
                hue = std::fmod(hue + rgb_timer.restart() / RGB_PERIOD, 1.0f);
                rgb.set_hsv(hue, 1, 0.2f);
            }
        }
    }
}
