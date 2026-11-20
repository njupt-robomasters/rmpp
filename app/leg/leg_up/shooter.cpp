#include "shooter.hpp"
#include "app.hpp"
#include <cmath>

void shooter_loop() {
    // N、S 挡使能，S 挡开摩擦轮；trigger 时 pitch 摇杆调弹频，鼠标左键按最高弹频
    shooter.set_enable(vt13.is_enable);
    shooter.set_bullet_speed(vt13.bullet_speed);
    shooter.set_bullet_freq(vt13.bullet_freq);
    shooter.update();

    // 摩擦轮关闭且低速时发零电流释放，避免 VESC 速度环在零速附近保持抖动
    if (vt13.bullet_speed == 0) {
        for (Vesc* rub : {&rub1, &rub2})
            if (std::fabs(rub->speed) < RUB_RELEASE_SPEED)
                rub->set_current(0);
    }
}
