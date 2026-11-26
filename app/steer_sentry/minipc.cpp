#include "minipc.hpp"
#include "gimbal.hpp"
#include "referee.hpp"

void minipc_loop() {
    aim.set_attitude(attitude.q);
    aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
    aim.update();

    // 上行大 yaw 相对底盘的编码器角，与导航下行的速度参考系配套
    nav.set_yaw_ecd(gimbal.yaw.ecd.measure);
    nav.update();
}
