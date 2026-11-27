#include "minipc.hpp"
#include "app.hpp"
#include "bsp/irq_lock.hpp"
#include "gimbal.hpp"

// 导航目标点坐标，m：中心增益点与补给区回家点
static constexpr float CENTER_X = 6.0f, CENTER_Y = -3.8f;
static constexpr float HOME_X = 0.15f, HOME_Y = -0.15f;

// 返航血量阈值：低于此值回家，回满上限血量后重新出击
static constexpr uint16_t GO_HOME_HP = 100;

// 2026 通信协议 V1.2.0 0x0209 bit23：RMUL 中心增益点 RFID
static constexpr uint32_t RFID_CENTER_RMUL = 1u << 23u;

void minipc_loop() {
    aim.update();
    // 超时会清零 Nav 的过洞标志，先保存最近有效状态供底盘和云台处理断联
    {
        const IrqLock lock;
        if (nav.is_connect)
            is_nav_through_hole = nav.is_through_hole;
        nav.update();
    }

    // 战术点位状态机：正赛或 fn 测试时生效；血量 <100 回家，回满上限血量再去中心，0x0209 bit23 判到达中心
    if (!sentry_in_game() && !fn_latched()) {
        nav_state = NavState::IDLE;
        nav.set_target(0, 0);
    } else {
        if (nav_state == NavState::IDLE)
            nav_state = NavState::GO_CENTER;

        const uint16_t max_hp = referee.robot_status.maximum_HP;
        const uint16_t hp = referee.robot_status.current_HP;
        if (max_hp != 0) {
            if (hp < GO_HOME_HP)
                nav_state = NavState::GO_HOME;
            else if (nav_state == NavState::GO_HOME && hp >= max_hp)
                nav_state = NavState::GO_CENTER;
        }

        if (nav_state != NavState::GO_HOME) {
            const bool is_at_center = referee.is_connect && (referee.rfid_status.rfid_status & RFID_CENTER_RMUL) != 0u;
            nav_state = is_at_center ? NavState::AT_CENTER : NavState::GO_CENTER;
        }

        if (nav_state == NavState::GO_HOME)
            nav.set_target(HOME_X, HOME_Y);
        else
            nav.set_target(CENTER_X, CENTER_Y);
    }

    aim.set_attitude(attitude.q);
    aim.set_referee(referee.robot_status.robot_id < 100, referee.shoot_data.initial_speed);
    nav.set_yaw_ecd(gimbal.yaw1.measure);
}
