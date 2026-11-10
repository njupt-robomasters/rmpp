// attitude 耗时测量：前 10 s 只调用 imu.update()，之后调用 attitude.update()
// SWD 观察 imu_cycles / attitude_cycles（单次周期数，max 为峰值，avg 为一阶平均）与 app_monitor
#include "attitude/quaternion_ekf.hpp"
#include "bsp/imu.hpp"
#include "bsp/monitor.hpp"
#include <cstdint>

// DWT CYCCNT（Timer 初始化时已开启），app 不包含 HAL 头，直接按地址读取
static volatile uint32_t& cyccnt = *reinterpret_cast<volatile uint32_t*>(0xE0001004);

struct Cycles {
    volatile uint32_t last = 0, max = 0;
    volatile float avg = 0;
};

static Imu imu({.temperature = 45.0f});
static QuaternionEkf attitude(imu, {});

static Cycles imu_cycles, attitude_cycles;
static Timer phase_timer;
static Monitor app_monitor;

static void record(Cycles& c, const uint32_t cycles) {
    c.last = cycles;
    if (cycles > c.max)
        c.max = cycles;
    c.avg = c.avg == 0 ? cycles : c.avg + 0.01f * (cycles - c.avg);
}

extern "C" void app_main() {
    phase_timer.restart();

    while (true) {
        // 1 kHz 更新
        static Timer app_timer;
        if (app_timer.elapsed() < 0.001f)
            continue;
        app_timer.restart();

        app_monitor.begin();
        const uint32_t start = cyccnt;
        if (phase_timer.elapsed() < 10.0f) {
            imu.update();
            record(imu_cycles, cyccnt - start);
        } else {
            attitude.update();
            record(attitude_cycles, cyccnt - start);
        }
        app_monitor.end();
    }
}
