#include "app.hpp"

// 解析遥控器控制
void handle_rc() {
    switch (dj6.left_switch) {
        case DJ6::ERR:
            shooter.SetPrepareShoot(false);
            shooter.SetShoot(false);
            break;
        case DJ6::UP:
            shooter.SetPrepareShoot(false);
            shooter.SetShoot(false);
            break;
        case DJ6::MID:
            shooter.SetPrepareShoot(true);
            shooter.SetShoot(false);
            break;
        case DJ6::DOWN:
            shooter.SetPrepareShoot(true);
            shooter.SetShoot(true);
            break;
    }
}

extern "C" void task_shooter_entry(void* argument) {
    shooter.SetBulletSpeed(18.0f * m_s);
    shooter.SetShootFreq(5.0f * Hz);
    while (true) {
        // 检查遥控器断联
        if (dj6.is_connected == false) {
            shooter.SetEnable(false); // 发射结构失能
        } else {
            shooter.SetEnable(true); // 发射机构使能
        }

        handle_rc();

        shooter.OnLoop();
        BSP::OS::Delay(1);
    }
}