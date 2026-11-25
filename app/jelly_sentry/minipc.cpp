#include "minipc.hpp"
#include "app.hpp"
#include "gimbal.hpp"

void minipc_loop() {
    aim.update();
    nav.update();
}

void minipc_send() {
    aim.set_attitude(attitude.q);
    aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
    aim.send();
}
