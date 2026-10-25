#include "imu/imu.hpp"

static Imu imu({
    .hardware = {.sensor = Bmi088({.accel = Spi(1, 1), .gyro = Spi(1, 2)}), .heater = Pwm(10, 1)},
    .adjust = {.gyro_offset = Vector3(0.00316276052f, -0.00411193585f, -0.00242328411f), .gravity = 9.79961872f},
});

extern "C" void app_main() {
    Timer timer;
    while (true) {
        if (timer.elapsed() > 0.001f) {
            timer.restart();
            imu.update();
        }
    }
}
