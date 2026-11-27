#include "shooter.hpp"
#include "app.hpp"
#include "minipc.hpp"
#include "sentry.hpp"

void shooter_loop() {
    shooter.set_enable(sentry_enable());

    if (sentry_active()) {
        sentry_shooter();
    } else if (sentry_official()) {
        // 自检停摩擦轮，5 s 倒计时预开摩擦轮，拨弹均停转
        const bool is_countdown = referee.game_status.game_progress == Referee::COUNTDOWN_5SEC;
        shooter.set_bullet_speed(is_countdown ? vt13.config.bullet_speed : 0);
        shooter.set_bullet_freq(0);
    } else {
        // VT13：S 挡开摩擦轮；人工自瞄需视觉开火许可，否则按本挡弹频
        shooter.set_bullet_speed(vt13.bullet_speed);
        shooter.set_bullet_freq(!vt13.is_aim || aim.is_fire ? vt13.bullet_freq : 0);
    }

    shooter.set_heat(referee.robot_status.shooter_barrel_heat_limit - referee.power_heat_data.shooter_17mm_barrel_heat,
                     referee.robot_status.shooter_barrel_cooling_value);
    shooter.update();
}
