#include "shooter.hpp"
#include "app.hpp"
#include "minipc.hpp"
#include "sentry.hpp"

void shooter_loop() {
    shooter.set_enable(vt13.is_enable);

    if (is_sentry_active()) {
        sentry_shooter();
    } else {
        shooter.set_bullet_speed(vt13.bullet_speed);
        if (vt13.is_aim && !aim.is_fire)
            shooter.set_bullet_freq(0);
        else
            shooter.set_bullet_freq(vt13.bullet_freq);
    }

    shooter.set_heat(referee.robot_status.shooter_barrel_heat_limit - referee.power_heat_data.shooter_17mm_barrel_heat,
                     referee.robot_status.shooter_barrel_cooling_value);
    shooter.update();
}
