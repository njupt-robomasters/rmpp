#include "app.hpp"
#include "math/math.hpp"
#include <algorithm>
#include <cmath>

// 操控参数
static constexpr float WR_MIN = 60 * rpm; // 导航满速平移时的小陀螺角速度

// 自主导航参数
static constexpr bool FORCE_GAME_POSITION = false;
static constexpr bool FORCE_TEST_POSITION = false;
static constexpr float GAME_CENTER_X = 4 - 2 + 0.2f, GAME_CENTER_Y = 6 - 1.5f + 0.2f;
static constexpr float GAME_HOME_X = 0, GAME_HOME_Y = 0;
static constexpr float TEST_CENTER_X = 1, TEST_CENTER_Y = 0;
static constexpr float TEST_HOME_X = 0, TEST_HOME_Y = 0;
static constexpr int GO_HOME_HP = 120, GO_CENTER_HP = 400;

enum ChassisState { GO_CENTER,
                    GO_HOME };

// 跨周期状态
static ChassisState chassis_state = GO_CENTER;

void chassis_loop() {
    chassis.set_enable(sentry_enable());
    chassis.set_mode(rc_fn() ? ChassisSteer::FOLLOW_MODE : ChassisSteer::DETACH_MODE);

    // 自主导航速度，按平移速度在最大转速和保留转速间插值小陀螺
    float vx = 0, vy = 0, wr = 0;
    if (sentry_active()) {
        const bool game_position = FORCE_GAME_POSITION || (!FORCE_TEST_POSITION && sentry_in_game());
        if (chassis_state == GO_CENTER) {
            nav.target_position.x = game_position ? GAME_CENTER_X : TEST_CENTER_X;
            nav.target_position.y = game_position ? GAME_CENTER_Y : TEST_CENTER_Y;
            if (referee.robot_status.current_HP <= GO_HOME_HP)
                chassis_state = GO_HOME;
        } else {
            nav.target_position.x = game_position ? GAME_HOME_X : TEST_HOME_X;
            nav.target_position.y = game_position ? GAME_HOME_Y : TEST_HOME_Y;
            if (referee.robot_status.current_HP >= GO_CENTER_HP)
                chassis_state = GO_CENTER;
        }
        std::tie(vx, vy) = rotate(nav.chassis_speed.vx, nav.chassis_speed.vy, yaw2.angle);
        const float speed = std::hypotf(vx, vy);
        wr = map(speed, 0.0f, vt13.config.vxy_max, vt13.config.wr_max, WR_MIN);
    } else {
        chassis_state = GO_CENTER;
        nav.target_position = {};
    }

    // 叠加遥控器与键盘，断联时为 0
    const float vxy_max = vt13.config.vxy_max, wr_max = vt13.config.wr_max;
    vx += vt13.vx;
    vy += vt13.vy;
    wr += vt13.wr;
    chassis.set_speed(std::clamp(vx, -vxy_max, vxy_max), std::clamp(vy, -vxy_max, vxy_max), std::clamp(wr, -wr_max, wr_max));

    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);
    super_cap.set_referee_limit(referee.robot_status.chassis_power_limit);
    super_cap.set_referee_energy(referee.power_heat_data.buffer_energy);
    super_cap.update();
    chassis.set_power_ref(super_cap.power_chassis_ref);
    chassis.update();
}
