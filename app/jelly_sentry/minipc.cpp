#include "minipc.hpp"
#include "app.hpp"
#include "gimbal.hpp"
#include <cmath>

void minipc_send() {
    aim.set_attitude(attitude.q);
    aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
    aim.send();

    // 导航上行：大 yaw 在线且角度有效时发送编码器实测角，rad；目标坐标暂取零
    if (yaw1.is_connect && std::isfinite(gimbal.yaw1.measure))
        nav.send(gimbal.yaw1.measure);
}
