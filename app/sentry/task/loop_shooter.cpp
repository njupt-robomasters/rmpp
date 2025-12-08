#include "app.hpp"

void loop_shooter() {
    // 检查遥控器断联
    if (dj6.is_connected == false) {
        shooter.SetEnable(false);
    } else {
        shooter.SetEnable(true);
    }

    // 设置弹速
    shooter.SetBulletSpeed(18.0f * m_s);

    // 设置单频
    shooter.SetShootFreq(5.0f * Hz);

    shooter.OnLoop();
}