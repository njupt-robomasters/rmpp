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
    sentry_update();
    chassis_loop();
    gimbal_loop();
    shooter_loop();
}

static void nav_cmd(const uint8_t is_through_hole) {
    const auto bytes = make_frame(1, 0, 0, 0, 0, is_through_hole);
    feed(2, bytes.data(), bytes.size());
}

int main() {
    // 非过洞无目标：底盘分离模式，云台正常扫描并在 pitch 编码器软限位间往返
    nav_cmd(0);
    gimbal.pitch.ecd_ref = 0;
    run();
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    CHECK(!gimbal.is_angle_cmd);
    near(gimbal.yaw_speed, 180 * deg_s);
    near(gimbal.pitch_speed, -60 * deg_s);

    gimbal.pitch.ecd_ref = PITCH_MIN;
    run();
    near(gimbal.pitch_speed, 60 * deg_s);

    // 过洞（折叠态）无目标：底盘切 FOLLOW_MODE，云台关闭自主扫描，避免随头追踪旋转扫描云台
    nav_cmd(1);
    run();
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    CHECK(!gimbal.is_angle_cmd);
    near(gimbal.yaw_speed, 0);
    near(gimbal.pitch_speed, 0);

    // 出洞恢复：保留原扫描方向继续扫描
    nav_cmd(0);
    run();
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    near(gimbal.yaw_speed, 180 * deg_s);
    near(gimbal.pitch_speed, 60 * deg_s);

    // 过洞时有效目标仍可锁定；脱锁超时或低血量后保持过洞零速停扫
    nav_cmd(1);
    run(true, 0.4f, -0.1f, 3.0f);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE && gimbal.is_angle_cmd);
    near(gimbal.angle_yaw, 0.4f);
    near(gimbal.angle_pitch, -0.1f);
    CHECK(gimbal.yaw_window);

    // 目标短暂消失未超时仍保持锁定；超过 500 ms 后退回过洞零速停扫
    now_us += 200000;
    nav_cmd(1);
    run(false);
    CHECK(gimbal.is_angle_cmd);

    for (int i = 0; i < 4; i++) {
        now_us += 90000;
        nav_cmd(1);
        run(false);
    }
    CHECK(!gimbal.is_angle_cmd);
    near(gimbal.yaw_speed, 0);
    near(gimbal.pitch_speed, 0);

    // 过洞低血量放弃锁定后同样保持零速停扫
    referee.robot_status.current_HP = 99;
    nav_cmd(1);
    run(true, 0.3f, 0, 2.0f);
    CHECK(!gimbal.is_angle_cmd);
    near(gimbal.yaw_speed, 0);
    near(gimbal.pitch_speed, 0);
    referee.robot_status.current_HP = 400;

    // 过洞期间人工拨杆优先，松手后恢复过洞停扫
    vt13.yaw_speed = 90 * deg_s;
    vt13.pitch_speed = 30 * deg_s;
    nav_cmd(1);
    run();
    CHECK(!gimbal.is_angle_cmd);
    near(gimbal.yaw_speed, 90 * deg_s);
    near(gimbal.pitch_speed, -30 * deg_s);
    vt13.yaw_speed = vt13.pitch_speed = 0;
    run();
    near(gimbal.yaw_speed, 0);
    near(gimbal.pitch_speed, 0);

    // 过洞断联：保存过洞状态，底盘停车；视觉与遥控输入均不启动自瞄或扫描
    now_us += 101000;
    vt13.yaw_speed = 90 * deg_s;
    run(true, 1.0f, 0.2f, 2.0f);
    CHECK(!nav.is_connect && !nav.is_through_hole && chassis.mode == ChassisMatrix::DETACH_MODE);
    CHECK(is_nav_through_hole && !gimbal.is_angle_cmd);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr_input, 0);
    near(gimbal.yaw_speed, 0);
    near(gimbal.pitch_speed, 0);
    vt13.yaw_speed = 0;
    run(true, 0.8f, 0.1f, 2.0f);
    CHECK(!gimbal.is_angle_cmd);
    near(gimbal.yaw_speed, 0);

    // 普通导航重新连接后恢复扫描，随后断联仍扫描并原地小陀螺
    nav_cmd(0);
    run();
    CHECK(!is_nav_through_hole);
    near(gimbal.yaw_speed, 180 * deg_s);
    near(gimbal.pitch_speed, 60 * deg_s);
    now_us += 101000;
    run();
    near(chassis.wr_input, 90 * rpm);
    near(gimbal.yaw_speed, 180 * deg_s);

    // 普通断联仍允许识别锁定，保留小 yaw 优先跟踪与人工接管
    run(true, 0.7f, -0.05f, 2.0f);
    CHECK(gimbal.is_angle_cmd && gimbal.yaw_window);
    near(gimbal.angle_yaw, 0.7f);
    vt13.yaw_speed = 30 * deg_s;
    run(true, 1.1f, 0, 2.0f);
    CHECK(!gimbal.is_angle_cmd);
    near(gimbal.yaw_speed, 30 * deg_s);

    // 战术点位：100 HP 仍可交战，99 HP 回家并屏蔽拨弹，满 maximum_HP 才出击
    vt13.yaw_speed = 0;
    referee.game_status.game_progress = Referee::GAMING;
    referee.is_connect = true;
    referee.robot_status.current_HP = 100;
    aim.is_fire = true;
    nav_cmd(0);
    run(true, 0.5f, 0, 2);
    CHECK(nav_state == NavState::GO_CENTER && gimbal.is_angle_cmd && gimbal.yaw_window);
    near(shooter.bullet_speed, 18);
    near(shooter.bullet_freq, 10);
    nav.send();
    float target[2];
    std::memcpy(target, cdc_tx[2].data() + 5, sizeof(target));
    near(target[0], 6);
    near(target[1], -3.8f);

    referee.robot_status.current_HP = 99;
    run(true, 0.7f, 0, 2);
    CHECK(nav_state == NavState::GO_HOME && !gimbal.is_angle_cmd && !gimbal.yaw_window);
    near(shooter.bullet_speed, 18);
    near(shooter.bullet_freq, 0);
    nav.send();
    std::memcpy(target, cdc_tx[2].data() + 5, sizeof(target));
    near(target[0], 0.15f);
    near(target[1], -0.15f);
    referee.robot_status.current_HP = 399;
    run();
    CHECK(nav_state == NavState::GO_HOME);
    referee.robot_status.current_HP = 400;
    referee.rfid_status.rfid_status = 1u << 23u;
    run();
    CHECK(nav_state == NavState::AT_CENTER);
    referee.rfid_status.rfid_status = 0;
    run();
    CHECK(nav_state == NavState::GO_CENTER);

    // 自检与倒计时：目标清零，自检停摩擦轮，倒计时预开，两者禁止拨弹
    test_active = false;
    referee.game_status.game_progress = Referee::REFEREE_SELF_CHECK;
    run(true, 0.9f, 0, 2);
    CHECK(nav_state == NavState::IDLE && !gimbal.is_angle_cmd);
    near(chassis.vx, 0);
    near(chassis.wr_input, 0);
    near(shooter.bullet_speed, 0);
    near(shooter.bullet_freq, 0);
    referee.game_status.game_progress = Referee::COUNTDOWN_5SEC;
    run(true, 0.9f, 0, 2);
    near(shooter.bullet_speed, 18);
    near(shooter.bullet_freq, 0);
    nav.send();
    std::memcpy(target, cdc_tx[2].data() + 5, sizeof(target));
    near(target[0], 0);
    near(target[1], 0);

    // 场下人工自瞄按识别启用窗口，发射条件采用上游
    referee.game_status.game_progress = Referee::NOT_STARTED;
    vt13.is_aim = true;
    vt13.bullet_speed = 18;
    vt13.bullet_freq = 10;
    aim.is_fire = false;
    run(true, 0.2f, 0, 2);
    CHECK(gimbal.is_angle_cmd && gimbal.yaw_window);
    near(shooter.bullet_freq, 0);
    aim.is_fire = true;
    run(true, 0.2f, 0, 2);
    near(shooter.bullet_freq, 10);
    vt13.rc.mode = VT13::C;
    vt13.is_enable = false;
    run(true, 0.2f, 0, 2);
    CHECK(!gimbal.enabled && !shooter.enabled && !chassis.enabled);

    CHECK(irq_depth == 0);
    std::printf("Production gimbal through-hole flow: %d checks passed\n", checks);
}
