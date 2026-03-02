#pragma once

#include "Infantry.hpp"

class Sentry : public Infantry {
public:
    using Infantry::Infantry;

    void OnLoop();

private:
    bool is_pause = false;

    enum {
        ORIGIN,
        POS1,
        POS2
    } nav_status = ORIGIN;

    void handleGame();
    void handleTest();
    void handlePause();
};