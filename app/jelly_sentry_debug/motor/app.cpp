#include "bsp/can.hpp"
#include "bsp/timer.hpp"
#include "motor/m3508.hpp"

// 单个 DJI 电机开环测试：CAN1 上电调 ID 为 5 的电机（反馈 0x205，控制帧 0x1FF 第 1 个位置）输出固定小电流
// 换电机时修改 rx_id 与控制帧 ID：ID 1~4 用 0x200，ID 5~8 用 0x1FF
// 在调试器中修改 current 调整输出电流，查看 motor.speed、motor.angle、motor.is_connect

volatile float current = 0.1f; // 输出电流，A

static M3508 motor({
    .can = Can(1),
    .rx_id = 0x205,
});

extern "C" void app_main() {
    motor.set_enable(true);

    Timer loop_timer;
    while (true) {
        if (loop_timer.elapsed() < 0.001f)
            continue;
        loop_timer.restart();

        motor.update();
        motor.set_current(current);

        CanFrame frame{.id = 0x1FF, .size = 8};
        motor.write_can_frame(frame);
        Can(1).send(frame);
    }
}
