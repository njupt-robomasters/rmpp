#include "attitude/quaternion_ekf.hpp"
#include "bsp/imu.hpp"
#include "bsp/timer.hpp"
#include <cmath>

// Imu：读取、恒温加热、零偏与安装方向修正，可单独使用，输出 imu.gyro / imu.accel / imu.temperature
static Imu imu({.gyro_offset = Vector3(0.00316276052f, -0.00411193585f, -0.00242328411f),
                .gravity = 9.79961872f,
                .temperature = 45.0f});

// 姿态解算：内部调用 imu.update()，输出 q、yaw/pitch/roll、yaw_total_angle
static QuaternionEkf attitude(imu);

extern "C" void app_main() {
    Timer timer, stable_timer;
    bool has_calibrated = false;
    while (true) {
        if (timer.elapsed() < 0.001f)
            continue;
        timer.restart();

        // 1 kHz 更新；只用 Imu 时改为调用 imu.update()
        attitude.update();

        // 温度在目标 ±1 °C 内持续 5 s 后校准一次零偏，期间保持静止；结果在 imu.config() 中查看并填回配置
        if (std::abs(imu.temperature - imu.config().temperature) > 1.0f)
            stable_timer.restart();
        if (!has_calibrated && stable_timer.elapsed() > 5.0f) {
            imu.start_calibration();
            has_calibrated = true;
        }
    }
}
