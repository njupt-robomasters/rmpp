#include "attitude/quaternion_ekf.hpp"
#include "bsp/imu.hpp"
#include "bsp/monitor.hpp"
#include "math/unit.hpp"

// 安装方向按实际修改；零偏与重力取 imu 示例校准结果
// orientation：传感器到机体的安装旋转，Z-Y-X 欧拉角，按实际安装修改
static Imu imu({.orientation = euler_to_quaternion({.yaw = 0 * deg, .pitch = 0 * deg, .roll = 0 * deg}),
                .gyro_offset = Vector3f(0.0f, 0.0f, 0.0f),
                .gravity = 9.81f});

// 内部调用 imu.update()；SWD 观察 attitude.q / yaw / pitch / roll / yaw_total_angle
static QuaternionEkf attitude(imu, {});

static void setup() {
    // imu.start_calibration();
}

static void loop() {
    attitude.update();
}

extern "C" void app_main() {
    setup();
    while (true) {
        // 1 kHz 更新
        static Timer app_timer;
        if (app_timer.elapsed() < 0.001f)
            continue;
        app_timer.restart();

        // 负载监控
        static Monitor app_monitor;
        app_monitor.begin();
        loop();
        app_monitor.end();
    }
}
