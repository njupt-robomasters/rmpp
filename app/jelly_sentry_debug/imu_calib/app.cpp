#include "attitude/quaternion_ekf.hpp"
#include "bsp/imu.hpp"
#include "bsp/rgb.hpp"
#include "bsp/timer.hpp"
#include "melody/melody.hpp"
#include "melody/music/c5d5g5.hpp"
#include "melody/music/g5g5g5.hpp"
#include <cmath>

// 哨兵陀螺仪零偏与重力模长校准，不驱动任何电机
// 使用方法：
// 1. 选择预设 jelly_sentry_debug-imu_calib，编译烧录
// 2. 上电后让主控（或整车）完全静止，校准过程中不要碰
// 3. 先等待 BMI088 加热到 TEMPERATURE 并稳定 STABLE_TIME（LED 黄色）
// 4. 温度稳定后响 C5D5G5，开始 5 s 采样（LED 蓝色）；非静止时 Imu 提前判为失败
// 5. 采样结束响 G5G5G5，LED 绿色；在调试器中查看 imu.calibration，为 SUCCEEDED 时
//    把 imu.gyro_offset、imu.gravity 填入 app/jelly_sentry/gimbal.hpp 的 Imu 配置
// 6. 静止时 attitude.yaw 基本不再漂移说明校准有效
// 注意：零偏随温度变化，整车程序的加热目标应与 TEMPERATURE 一致；更换主控或 IMU 后需要重新校准

static constexpr float TEMPERATURE = 50.0f;          // 恒温目标，℃
static constexpr float TEMPERATURE_TOLERANCE = 0.5f; // 认为温度稳定的误差范围，℃
static constexpr float STABLE_TIME = 5;              // 温度持续稳定多久后开始采样，s
static constexpr float CALIBRATION_TIME = 5.5f;      // 等待 Imu 完成 5 s 采样的时间，s

// 校准在传感器系进行，与安装方向无关；attitude 仅用于观察校准效果
static Imu imu({.temperature = TEMPERATURE});
static QuaternionEkf attitude(imu, {});
static Rgb rgb;
static Melody melody({});

enum CalibState { PREHEAT,
                  CALIBRATE,
                  DONE };

// 跨周期状态，可在调试器中观察
static CalibState calib_state = PREHEAT;
static Timer stable_timer, calibrate_timer;

extern "C" void app_main() {
    Timer loop_timer;
    while (true) {
        if (loop_timer.elapsed() < 1 * ms)
            continue;
        loop_timer.restart();

        // 内部调用 imu.update()，运行恒温控制与校准
        attitude.update();
        melody.update();

        switch (calib_state) {
        case PREHEAT:
            rgb.set(1, 1, 0);
            if (std::fabs(imu.temperature - TEMPERATURE) > TEMPERATURE_TOLERANCE)
                stable_timer.restart();
            if (stable_timer.elapsed() > STABLE_TIME) {
                imu.start_calibration();
                melody.play(C5D5G5);
                calibrate_timer.restart();
                calib_state = CALIBRATE;
            }
            break;
        case CALIBRATE:
            rgb.set(0, 0, 1);
            if (calibrate_timer.elapsed() > CALIBRATION_TIME) {
                melody.play(G5G5G5);
                calib_state = DONE;
            }
            break;
        case DONE:
            rgb.set(0, 1, 0);
            break;
        }
    }
}
