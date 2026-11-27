#include "minipc.hpp"
#include "app.hpp"
#include "gimbal.hpp"

void minipc_loop() {
    aim.set_attitude(attitude.q);
    aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
    aim.update();

    nav.set_yaw_ecd(gimbal.yaw1.measure);
    nav.update();
}
