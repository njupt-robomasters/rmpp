#include "shooter.hpp"
#include "app.hpp"
#include "minipc.hpp"
#include "referee.hpp"

void shooter_loop() {
    // N、S 挡使能，S 挡开摩擦轮；自瞄时需视觉批准，弹频始终由 VT13 操作给出
    shooter.set_enable(vt13.is_enable);
    shooter.set_bullet_speed(vt13.bullet_speed);
    if (vt13.is_aim) {
        if (aim.is_detect && aim.is_fire)
            shooter.set_bullet_freq(vt13.bullet_freq);
        else
            shooter.set_bullet_freq(0.0f);
    } else {
        shooter.set_bullet_freq(vt13.bullet_freq);
    }
    shooter.set_heat_remain(referee.robot_status.shooter_barrel_heat_limit - referee.power_heat_data.shooter_17mm_barrel_heat);
    shooter.update();
}
