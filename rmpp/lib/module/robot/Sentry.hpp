#pragma once

#include "Infantry.hpp"

class Sentry : public Infantry {
public:
    using Infantry::Infantry;

    void OnLoop();

private:
    enum {
        RC_MODE,
        DEBUG_MODE,
        GAME_MODE,
    } mode = RC_MODE;

    void handleMode();

    void handleDebug();

    void handleGame();
};