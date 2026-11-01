#include "app.hpp"
#include "bsp/timer.hpp"

extern "C" void app_main() {
    // 上电静止校准零偏与重力模长，约 6 s；外部工程同样每次上电校准
    imu.start_calibration();

    Timer timer;
    bool is_control_tick = false;
    while (true) {
        if (timer.elapsed() < 1 * ms)
            continue;
        timer.restart();

        // 1 kHz 更新遥控与姿态
        fsi6x.update();
        attitude.update();

        // CAN2 上 4 个关节各带一帧回复，底盘控制与发送分两个 1 ms 周期，控制频率 500 Hz
        // 每周期不超过 3 帧，匹配 3 个发送邮箱；外部工程周期同为约 2 ms
        is_control_tick = !is_control_tick;
        if (is_control_tick) {
            chassis_loop();

            left_e.send();
            left_a.send();
            CanFrame frame{.id = 0x200, .size = 8};
            left_wheel.write_can_frame(frame);
            right_wheel.write_can_frame(frame);
            Can(2).send(frame);
        } else {
            right_e.send();
            right_a.send();
        }
    }
}
