#pragma once

#include "Infantry.hpp"

class Sentry : public Infantry {
public:
    using Infantry::Infantry;

    void OnLoop();

private:
    bool is_ignore_mavlink = false;

    void handleGame();

    void handleTest();
};