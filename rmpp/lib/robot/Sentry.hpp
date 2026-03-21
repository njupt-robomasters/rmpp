#pragma once

#include "Robot.hpp"

class Sentry : public Robot {
public:
    using Robot::Robot;

    void OnLoop();

private:
    struct pos_t {
        UnitFloat<m> x, y;
    };

    // 家坐标
    static constexpr pos_t GAME_HOME = {0 * m, 0 * m};
    static constexpr UnitFloat<m> GAME_HOME_R = 0.5 * m;
    static constexpr pos_t TEST_HOME = {0 * m, 0 * m};
    static constexpr UnitFloat<m> TEST_HOME_R = 0.2 * m;

    // 中心点坐标
    static constexpr pos_t GAME_CENTER = {4 * m, 6 * m};
    static constexpr UnitFloat<m> GAME_CENTER_R = 1 * m;
    static constexpr pos_t TEST_CENTER = {1 * m, 0 * m};
    static constexpr UnitFloat<m> TEST_CENTER_R = 0.2 * m;

    // 小陀螺转速
    static constexpr UnitFloat<rpm> GO_WR = 60 * rpm;
    static constexpr UnitFloat<rpm> HOME_WR = 60 * rpm;
    static constexpr UnitFloat<rpm> CENTER_WR = 120 * rpm;

    // 云台速度
    static constexpr UnitFloat<deg_s> YAW_SPEED = 180 * deg_s;
    static constexpr UnitFloat<deg_s> PITCH_SPEED = 60 * deg_s;
    static constexpr Angle<deg> PITCH_MIN = -15 * deg;
    static constexpr Angle<deg> PITCH_MAX = 0 * deg;

    bool is_game = false;

    pos_t home = GAME_HOME;
    UnitFloat<m> home_r = GAME_HOME_R;
    pos_t center = GAME_CENTER;
    UnitFloat<m> center_r = GAME_CENTER_R;

    enum {
        GO_CENTER,
        IN_CENTER,
        GO_HOME,
        IN_HOME,
    } status = GO_CENTER;

    bool checkPos(const pos_t& pos, const UnitFloat<>& r);

    void handlePause();

    void handleAutoAim();

    void handleGame();
};