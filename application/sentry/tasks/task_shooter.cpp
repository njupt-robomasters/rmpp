#include "app.hpp"

void handle_rc() {
    static DJ6::switch_e last;

    if (dj6.left_switch != last) {
        last = dj6.left_switch;

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

void handle_video() {
    static bool last;

    if (referee.mouse_left_button_down != last) {
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
    while (true) {
        if (dj6.is_connected == false) {
            shooter.SetEnable(false);
            shooter.Update();
            BSP::OS::Delay(1);
            continue;
        }

        handle_rc(); // 解析遥控器
        handle_video(); // 解析图传链路键盘鼠

        shooter.Update();
        BSP::OS::Delay(1);
    }
}
