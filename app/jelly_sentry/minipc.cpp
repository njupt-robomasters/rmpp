#include "minipc.hpp"
#include "app.hpp"
#include "gimbal.hpp"

void minipc_loop() {
    aim.update();
    nav.update();

    aim.set_attitude(attitude.q);
    aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
    nav.set_yaw_ecd(gimbal.yaw1.measure);
}
