#pragma once

#include "Infantry.hpp"

class Sentry : public Infantry {
public:
    using Infantry::Infantry;

    void OnLoop();

private:
    struct pos_t {
        UnitFloat<m> x, y;
    };

    static constexpr pos_t HOME = {0 * m, 0 * m};
    static constexpr UnitFloat<m> HOME_R = 1 * m;
    static constexpr pos_t CENTER = {4 * m, 6 * m};
    static constexpr UnitFloat<m> CENTER_R = 1 * m;

    bool is_pause = false;

    enum {
        GO_CENTER,
        IN_CENTER,
        GO_HOME,
        IN_HOME,
    } game_status = GO_CENTER;

    enum {
        GO_ORIGIN,
        GO_POS1,
        GO_POS2
    } test_status = GO_ORIGIN;

    bool checkPos(const pos_t& pos, const UnitFloat<>& r);

    void handlePause();

    void handleGame();

    void handleTest();
};