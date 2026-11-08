#include "referee/referee.hpp"

// 哨兵裁判系统测试：在调试器中查看 referee 的各成员与 is_connected
// 与整车一致，经 CAN1 转发模块接入

static Referee referee({.bus = Can(1)});

extern "C" void app_main() {
    Timer loop_timer;
    while (true) {
        if (loop_timer.elapsed() > 0.001f) {
            loop_timer.restart();
            referee.update();
        }
    }
}
