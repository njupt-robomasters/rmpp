#pragma once

#include "Robot.hpp"
#include "math/Angle.hpp"
#include "math/Conversions.hpp"

using math::Angle;
using math::deg;
using math::deg_s;
using math::ms;

class Sentry : public Robot {
public:
    using Robot::Robot;

    void OnLoop();

private:
    static constexpr bool force_game_position = false;
    static constexpr bool force_test_position = false;

    // 坐标
    // 比赛模式坐标
    static constexpr float GAME_CENTER_X = (4 - 2 + 0.2);
    static constexpr float GAME_CENTER_Y = (6 - 1.5 + 0.2);
    static constexpr float GAME_HOME_X = 0;
    static constexpr float GAME_HOME_y = 0;
    // 调试模式坐标
    static constexpr float TEST_CENTER_X = 1;
    static constexpr float TEST_CENTER_Y = 0;
    static constexpr float TEST_HOME_X = 0;
    static constexpr float TEST_HOME_y = 0;

    // 动作血量
    static constexpr auto GO_HOME_HP = 120;   // 回家血量
    static constexpr auto GO_CENTER_HP = 400; // 去中心点血量

    // 云台参数
    static constexpr float YAW_SPEED = 180 * deg_s;
    static constexpr float PITCH_SPEED = 60 * deg_s;
    static constexpr Angle PITCH_MIN = -22 * deg;
    static constexpr Angle PITCH_MAX = 0 * deg;

    // 自瞄参数
    static constexpr float LOCK_DISTANCE = 5;          // 自瞄有效距离
    static constexpr float LOCK_TIMEOUT = 500 * ms;    // 自瞄脱锁时间
    static constexpr float INSTA360_CHECK_TIMEOUT = 1; // inst360检测时间
    static constexpr float INSTA360_PUNISH_TIME = 0;   // insta360识别错误惩罚时间

    bool is_game = false;

    // 底盘状态机
    enum {
        GO_CENTER,
        GO_HOME,
    } chassis_status = GO_CENTER;

    // 云台状态机
    enum {
        SCAN,
        INSTA360_CHECK,
        LOCK,
    } gimbal_status = SCAN;

    BSP::Dwt dwt_lock_lost;
    BSP::Dwt dwt_insta360_check;
    BSP::Dwt dwt_insta360_punish;

    void handlePause();

    void handle15s();

    void handleChassis();

    void handleGimbal();

    void handleShooter();
};