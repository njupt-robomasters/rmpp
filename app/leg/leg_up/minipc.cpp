#include "minipc.hpp"
#include "gimbal.hpp"
#include "referee.hpp"

void minipc_loop() {
    aim.set_attitude(attitude.q);
    aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
    aim.update();
}
