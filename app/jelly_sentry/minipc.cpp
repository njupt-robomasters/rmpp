#include "minipc.hpp"
#include "app.hpp"
#include "bsp/irq_lock.hpp"
#include "gimbal.hpp"

void minipc_loop() {
    aim.update();
    // 超时会清零 Nav 的过洞标志，先保存最近有效状态供底盘和云台处理断联
    {
        const IrqLock lock;
        if (nav.is_connect)
            is_nav_through_hole = nav.is_through_hole;
        nav.update();
    }

    aim.set_attitude(attitude.q);
    aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
    nav.set_yaw_ecd(gimbal.yaw1.measure);
}
