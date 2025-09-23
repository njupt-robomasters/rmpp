#include "app.hpp"

void handle_rc() {
    static DJ6::switch_e last;

    if (dj6.left_switch != last) { // 只响应变化量
        last = dj6.left_switch;

        if (last == DJ6::ERR) // 遥控器第一次连上，不响应
            return;

        switch (dj6.left_switch) {
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
}

void handle_referee() {
    static bool last;

    if (referee.mouse_left_button_down != last) { // 只响应变化量
        last = referee.mouse_left_button_down;

        if (referee.mouse_left_button_down) {
            shooter.SetPrepareShoot(true);
            shooter.SetShoot(true);
        } else {
            shooter.SetShoot(false);
        }
    }
}

extern "C" void task_shooter_entry(const void* argument) {
    shooter.SetBulletSpeed(24.0f);
    shooter.SetShootFreq(10.0f);
    while (true) {
        handle_rc(); // 解析遥控器
        handle_referee(); // 解析图传链路键盘鼠

        // 检查遥控器断联
        if (dj6.is_connected == false) {
            shooter.SetEnable(false); // 发射结构失能
            shooter.Update();
            BSP::OS::Delay(1);
            continue;
        }

        shooter.SetEnable(true); // 发射机构使能

        shooter.Update();
        BSP::OS::Delay(1);
    }
}
