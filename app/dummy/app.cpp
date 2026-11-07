#include "app.hpp"
#include "chassis.hpp"
#include "gimbal.hpp"
#include "melody/music/c5d5g5.hpp"

extern "C" void app_main() {
    melody.play(C5D5G5);

    while (true) {
        // 1 kHz 更新
        static Timer timer;
        if (timer.elapsed() < 0.001f)
            continue;
        timer.restart();

        melody.update();
        fsi6x.update();

        chassis_loop();
        // gimbal_loop();
    }
}
