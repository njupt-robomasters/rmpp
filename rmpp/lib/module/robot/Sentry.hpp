#pragma once

#include "Infantry.hpp"

class Sentry : public Infantry {
public:
    using Infantry::Infantry;

    void OnLoop();

private:

    enum {
        ORIGIN,
        POS1,
        POS2
    } nav_status = ORIGIN;

    void handleDebug();

    void handleChassis() override;
};