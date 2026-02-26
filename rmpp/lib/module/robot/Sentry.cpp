#include "Sentry.hpp"

void Sentry::OnLoop() {
    Infantry::OnLoop();

    handleMode();
    handleDebug();
    handleGame();
}

void Sentry::handleMode() {
    if (device.referee.game.progress == Referee::COUNTDOWN_5SEC || device.referee.game.progress == Referee::GAMING) {
        mode = GAME_MODE;
    } else {
        if (device.rc.is_enable == false) {
            mode = RC_MODE;
        }

        if (device.rc.is_enable && device.rc.vt13.pause) {
            mode = DEBUG_MODE;
        }
    }
}

void Sentry::handleDebug() {

}

void Sentry::handleGame() {

}