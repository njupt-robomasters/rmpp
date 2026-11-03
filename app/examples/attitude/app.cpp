#include "attitude/quaternion_ekf.hpp"
#include "bsp/imu.hpp"
#include "bsp/timer.hpp"
#include "math/unit.hpp"

// 安装方向按实际修改；零偏与重力取 imu 示例校准结果
// orientation：传感器到机体的安装旋转，Z-Y-X 欧拉角，按实际安装修改
static Imu imu({.orientation = euler_to_quaternion({.yaw = 0 * deg, .pitch = 0 * deg, .roll = 0 * deg}),
                .gyro_offset = Vector3(-0.00637f, 0.00098f, 0.00191f),
                .gravity = 9.66301f,
                .temperature = 45.0f});

// 内部调用 imu.update()；SWD 观察 attitude.q / yaw / pitch / roll / yaw_total_angle
static QuaternionEkf attitude(imu, {});

extern "C" void app_main() {
    // imu.start_calibration();
    while (true) {
        // 1 kHz 更新
        static Timer timer;
        if (timer.elapsed() < 0.001f)
            continue;
        timer.restart();

        attitude.update();
    }
}
