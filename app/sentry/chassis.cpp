#include "app.hpp"
#include "math/math.hpp"
#include <cmath>

static constexpr bool FORCE_GAME_POSITION = false;
static constexpr bool FORCE_TEST_POSITION = false;
static constexpr float GAME_CENTER_X = 4 - 2 + 0.2f, GAME_CENTER_Y = 6 - 1.5f + 0.2f;
static constexpr float GAME_HOME_X = 0, GAME_HOME_Y = 0;
static constexpr float TEST_CENTER_X = 1, TEST_CENTER_Y = 0;
static constexpr float TEST_HOME_X = 0, TEST_HOME_Y = 0;
static constexpr int GO_HOME_HP = 120, GO_CENTER_HP = 400;

enum ChassisState { GO_CENTER,
                    GO_HOME };

static ChassisState chassis_state = GO_CENTER;

void chassis_loop() {
    chassis.set_enable(rc.is_enable);
    if (rc.is_fn)
        chassis.set_mode(Chassis::FOLLOW_MODE);
    float vx = 0, vy = 0, wr = 0;
    if (sentry_active()) {
        const bool game_position = FORCE_GAME_POSITION || (!FORCE_TEST_POSITION && sentry_in_game());
        if (chassis_state == GO_CENTER) {
            mavlink.target_position.x = game_position ? GAME_CENTER_X : TEST_CENTER_X;
            mavlink.target_position.y = game_position ? GAME_CENTER_Y : TEST_CENTER_Y;
            if (referee.robot.hp <= GO_HOME_HP)
                chassis_state = GO_HOME;
        } else {
            mavlink.target_position.x = game_position ? GAME_HOME_X : TEST_HOME_X;
            mavlink.target_position.y = game_position ? GAME_HOME_Y : TEST_HOME_Y;
            if (referee.robot.hp >= GO_CENTER_HP)
                chassis_state = GO_CENTER;
        }
        std::tie(vx, vy) = rotate(mavlink.chassis_speed.vx, mavlink.chassis_speed.vy, yaw2.position.measure);
        const float speed = std::sqrtf(square(vx) + square(vy));
        wr = map(speed, 0.0f, rc.config.vxy_max, rc.config.wr_max, chassis.config.reserve_wr);
    } else {
        chassis_state = GO_CENTER;
        mavlink.target_position = {};
    }
    chassis.set_speed(clamp(rc.vx + vx, rc.config.vxy_max), clamp(rc.vy + vy, rc.config.vxy_max), clamp(rc.wr + wr, rc.config.wr_max));
    chassis.set_gimbal_yaw(gimbal.yaw.ecd.measure);
    chassis.set_power_limit(referee.chassis.power_limit);
    chassis.set_buffer_energy(referee.chassis.buffer_energy);
    chassis.update();
}
