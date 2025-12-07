#include "app.hpp"

extern "C" void task_shooter_entry(void* argument) {
    shooter.SetBulletSpeed(18.0f * m_s);
    shooter.SetShootFreq(5.0f * Hz);
    while (true) {
        // 检查遥控器断联
        if (dj6.is_connected == false) {
            shooter.SetEnable(false);
        } else {
            shooter.SetEnable(true);
        }

        shooter.OnLoop();

        BSP::OS::Delay(1);
    }
}