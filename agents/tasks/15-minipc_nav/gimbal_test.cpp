#include "chassis.cpp"
#include "gimbal.cpp"

static void run(const bool is_detect = false, const Angle target_yaw = 0, const Angle target_pitch = 0, const float distance = 10) {
    now_us += 1000;
    nav.update();
    chassis_loop();
    gimbal_loop(is_detect, target_yaw, target_pitch, distance);
}

static void nav_cmd(const uint8_t is_through_hole) {
    const auto bytes = make_frame(1, 0, 0, 0, 0, is_through_hole);
    feed(2, bytes.data(), bytes.size());
}

int main() {
    // 非过洞无目标：底盘分离模式，云台正常扫描并在 pitch 编码器软限位间往返
    nav_cmd(0);
    gimbal.pitch.ecd.ref = 0;
    run();
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    CHECK(!gimbal.is_angle_cmd);
    near(gimbal.yaw_speed, 180 * deg_s);
    near(gimbal.pitch_speed, -60 * deg_s);

    gimbal.pitch.ecd.ref = PITCH_MIN;
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
    referee.robot_status.current_HP = 100;
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

    // 导航超时断联后过洞标志自动清零，云台恢复扫描、底盘回分离模式
    now_us += 101000;
    run();
    CHECK(!nav.is_connect && !nav.is_through_hole && chassis.mode == ChassisMatrix::DETACH_MODE);
    near(gimbal.yaw_speed, 180 * deg_s);
    near(gimbal.pitch_speed, 60 * deg_s);

    CHECK(irq_depth == 0);
    std::printf("Production gimbal through-hole flow: %d checks passed\n", checks);
}
