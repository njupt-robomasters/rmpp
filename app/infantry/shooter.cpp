#include "app.hpp"

void shooter_loop() {
    shooter.set_enable(rc.is_enable);
    const auto command = rc.shoot_command(mavlink.vision.is_fire);
    shooter.set_rub(rc.is_rub);
    shooter.set_shoot(command.is_shoot, command.rate);
    if (referee.robot.type == Referee::INFANTRY)
        shooter.adjust_bullet(rc.bullet_step);
    shooter.set_heat_remain(referee.shooter.heat_remain);
    shooter.update();
}
