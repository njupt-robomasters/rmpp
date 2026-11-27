#include "chassis.cpp"
#include "gimbal.cpp"
#include "minipc.cpp"
#include "sentry.cpp"
#include "shooter.cpp"

static void run(const bool is_detect = false, const Angle target_yaw = 0, const Angle target_pitch = 0, const float distance = 10) {
    now_us += 1000;
    aim.is_detect = is_detect;
    aim.yaw = target_yaw;
    aim.pitch = target_pitch;
    aim.distance = distance;
    vt13.rc.fn = test_active;
    vt13.rc.pause = !test_active;
    minipc_loop();
    chassis_loop();
    gimbal_loop();
    shooter_loop();
}

static void nav_cmd() {
    const auto bytes = make_frame(0, 0, 0, 0, 0, 0);
    feed(2, bytes.data(), bytes.size());
}

int main() {
    // 无目标：云台正常扫描并在 pitch 编码器软限位间往返
    nav_cmd();
    gimbal.pitch.ecd_ref = 0;
    run();
    CHECK(!gimbal.is_angle_cmd);
    near(gimbal.yaw_speed, 180 * deg_s);
    near(gimbal.pitch_speed, -60 * deg_s);

    // 到端点换倍率：pitch 反向且速度不超过 60°/s
    gimbal.pitch.ecd_ref = PITCH_MIN;
    run();
    CHECK(gimbal.pitch_speed > 0 && gimbal.pitch_speed <= 60 * deg_s && gimbal.pitch_speed >= 30 * deg_s);

    // 5 m 内识别到目标：锁定视觉角度并启用小 yaw 优先窗口
    run(true, 0.4f, -0.1f, 3.0f);
    CHECK(gimbal.is_angle_cmd && gimbal.yaw_window);
    near(gimbal.angle_yaw, 0.4f);
    near(gimbal.angle_pitch, -0.1f);

    // 目标短暂消失未超时仍保持锁定，窗口跟随锁定一起保持；超过 500 ms 后退回扫描并关闭窗口
    now_us += 200000;
    nav_cmd();
    run(false);
    CHECK(gimbal.is_angle_cmd && gimbal.yaw_window);
    for (int i = 0; i < 6; i++) {
        now_us += 90000;
        nav_cmd();
        run(false);
    }
    CHECK(!gimbal.is_angle_cmd && !gimbal.yaw_window);
    near(gimbal.yaw_speed, 180 * deg_s);

    // 超出 5 m 不锁定，扫描由大 yaw 承担，不进入小 yaw 优先窗口
    run(true, 0.3f, 0, 6.0f);
    CHECK(!gimbal.is_angle_cmd && !gimbal.yaw_window);

    // 导航断联：底盘原地 90 rpm，云台继续扫描
    now_us += 101000;
    run();
    CHECK(!nav.is_connect);
    near(chassis.wr_input, vt13.config.wr_max);
    near(gimbal.yaw_speed, 180 * deg_s);

    // 战术点位：100 HP 可交战，99 HP 回家并屏蔽拨弹，回满 maximum_HP 再出击；回家点按红蓝方区分，中心点共用
    test_active = false;
    referee.game_status.game_progress = Referee::GAMING;
    referee.robot_status.robot_id = 7; // 红方
    referee.robot_status.current_HP = 100;
    aim.is_fire = true;
    nav_cmd();
    run(true, 0.5f, 0, 2);
    CHECK(!is_go_home && gimbal.is_angle_cmd && gimbal.yaw_window);
    near(shooter.bullet_speed, 18);
    near(shooter.bullet_freq, 10);
    nav.send();
    float target[2];
    std::memcpy(target, cdc_tx[2].data() + 5, sizeof(target));
    near(target[0], 6);
    near(target[1], -3.8f);

    // 红方回家点
    referee.robot_status.current_HP = 99;
    nav_cmd();
    run(true, 0.7f, 0, 2);
    CHECK(is_go_home && !gimbal.is_angle_cmd && !gimbal.yaw_window);
    near(shooter.bullet_speed, 18);
    near(shooter.bullet_freq, 0);
    nav.send();
    std::memcpy(target, cdc_tx[2].data() + 5, sizeof(target));
    near(target[0], -10.160f);
    near(target[1], 4.663f);

    // 蓝方回家点
    referee.robot_status.robot_id = 107;
    nav_cmd();
    run();
    nav.send();
    std::memcpy(target, cdc_tx[2].data() + 5, sizeof(target));
    near(target[0], 0.15f);
    near(target[1], -0.15f);

    // 未收到最大血量：锁存保持，不再逐周期翻转
    referee.robot_status.maximum_HP = 0;
    nav_cmd();
    run();
    CHECK(is_go_home);
    nav_cmd();
    run();
    CHECK(is_go_home);

    // 回满 maximum_HP 再出击，出击目标中心点红蓝共用
    referee.robot_status.maximum_HP = 400;
    referee.robot_status.current_HP = 399;
    nav_cmd();
    run();
    CHECK(is_go_home);
    referee.robot_status.current_HP = 400;
    nav_cmd();
    run();
    CHECK(!is_go_home);
    nav.send();
    std::memcpy(target, cdc_tx[2].data() + 5, sizeof(target));
    near(target[0], 6);
    near(target[1], -3.8f);

    // 测试模式：摩擦轮与开火都要手动 S 挡
    referee.game_status.game_progress = Referee::NOT_STARTED;
    test_active = true;
    aim.is_fire = true;
    nav_cmd();
    run(true, 0.2f, 0, 2);
    near(shooter.bullet_speed, 0);
    near(shooter.bullet_freq, 0);
    vt13.rc.mode = VT13::S;
    vt13.bullet_speed = 18;
    vt13.bullet_freq = 10;
    run(true, 0.2f, 0, 2);
    near(shooter.bullet_speed, 18);
    near(shooter.bullet_freq, 10);

    // C 挡失能
    vt13.rc.mode = VT13::C;
    vt13.is_enable = false;
    run(true, 0.2f, 0, 2);
    CHECK(!gimbal.enabled && !shooter.enabled && !chassis.enabled);

    CHECK(irq_depth == 0);
    std::printf("Production gimbal flow: %d checks passed\n", checks);
}
