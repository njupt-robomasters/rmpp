#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"

// 哨兵遥控器测试：在调试器中查看 vt13、fsi6x 的摇杆、拨杆、键鼠与 is_connect
// 接线与整车一致：VT13 图传接 USART1（921600），FSi6X 接收机 SBUS 接 UART5；关闭遥控器 0.5 s 后判定断联

static VT13 vt13({.uart = Uart(1)});
static FSi6X fsi6x({.uart = Uart(5)});

extern "C" void app_main() {
    Timer loop_timer;
    while (true) {
        if (loop_timer.elapsed() > 0.001f) {
            loop_timer.restart();
            vt13.update();
            fsi6x.update();
        }
    }
}
