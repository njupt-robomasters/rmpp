#include "Sentry.hpp"

void Sentry::OnLoop() {
    Infantry::OnLoop();

    // 按下停止键忽略上位机指令
    if (device.rc.vt13.pause) {
        is_ignore_mavlink = true;
    }

    // 失能释放忽略上位机指令
    if (device.rc.is_enable == false) {
        is_ignore_mavlink = false;
    }

    if (is_ignore_mavlink == false) {
        if (device.referee.game.progress == Referee::GAMING) {
            handleGame();
        } else {
            handleTest();
        }
    }
}

void Sentry::handleGame() {}

void Sentry::handleTest() {}