#include "chassis.cpp"
#include "minipc.cpp"

static void run() {
    now_us += 1000;
    minipc_loop();
    chassis_loop();
}

static void command(const float vx, const float vy, const uint8_t is_through_hole) {
    const auto bytes = make_frame(vx, vy, 0.7f, 0, 0, is_through_hole);
    feed(2, bytes.data(), bytes.size());
    run();
}

int main() {
    run();
    CHECK(chassis.enabled && chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr_input, 90 * rpm);

    // 雷达朝前（yaw1 = 0）、云台朝左（yaw2 = +90°）：导航向前 1 m/s 在头部系为向右 1 m/s，落到车身系为向前 1 m/s
    gimbal.yaw2.measure = 90 * deg;
    gimbal.yaw.ecd.measure = 90 * deg;
    command(1, 0, 1);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    near(chassis.vx, 0);
    near(chassis.vy, -1);
    const auto [body_vx, body_vy] = rotate(chassis.vx, chassis.vy, chassis.gimbal_yaw);
    near(body_vx, 1);
    near(body_vy, 0);
    near(chassis.wr_input, 0);
    near(chassis.gimbal_yaw, 90 * deg);

    // 同一姿态下遥控向前 1 m/s 仍沿头部朝向（车身向左），与导航向前合成后车身为前 1、左 1
    vt13.vx = 1;
    command(1, 0, 1);
    const auto [mix_body_vx, mix_body_vy] = rotate(chassis.vx, chassis.vy, chassis.gimbal_yaw);
    near(mix_body_vx, 1);
    near(mix_body_vy, 1);
    vt13.vx = 0;

    // 任意小 yaw 偏转下，纯导航速度经两级旋转后始终等于仅按大 yaw 转到车身系的结果
    for (int i = -10; i <= 10; i++) {
        const float large_yaw = i * 0.2f;
        gimbal.yaw2.measure = i * 0.06f;
        gimbal.yaw.ecd.measure = large_yaw + float(gimbal.yaw2.measure);
        command(1.2f, -0.8f, 1);
        const auto [actual_bx, actual_by] = rotate(chassis.vx, chassis.vy, chassis.gimbal_yaw);
        const auto [expect_bx, expect_by] = rotate(1.2f, -0.8f, large_yaw);
        near(actual_bx, expect_bx);
        near(actual_by, expect_by);
        near(chassis.gimbal_yaw, gimbal.yaw.ecd.measure);
    }

    gimbal.yaw2.measure = 0;
    // 非过洞：按合速度在 90→60 rpm 插值；超量程平移保留，只限制旋转插值输入
    command(0, 0, 0);
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.wr_input, 90 * rpm);
    vt13.vy = -2;
    command(1.5f, 0, 0);
    near(chassis.wr_input, 75 * rpm);
    vt13.vy = 0;
    command(5, 0, 0);
    near(chassis.wr_input, 60 * rpm);
    command(100, 0, 0);
    near(chassis.vx, 100);
    near(chassis.wr_input, 60 * rpm);
    command(0, 0, 1);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    near(chassis.wr_input, 0);

    // 导航向前、摇杆向右：两个方向都有速度，松杆后只剩导航
    vt13.vy = -2;
    command(2, 0, 1);
    near(chassis.vx, 2);
    near(chassis.vy, -2);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    vt13.vy = 0;
    run();
    near(chassis.vx, 2);
    near(chassis.vy, 0);

    // 同向相加保留超出遥控满量程的平移目标；反向抵消后恢复原地 90 rpm
    vt13.vx = 2;
    vt13.vy = -2;
    command(4, -4, 0);
    near(chassis.vx, 6);
    near(chassis.vy, -6);
    near(chassis.wr_input, 60 * rpm);
    command(-2, 2, 0);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr_input, 90 * rpm);
    vt13.vx = vt13.vy = 0;

    // 人工模式取遥控速度，切回自主模式恢复导航
    test_active = false;
    vt13.vx = 1;
    vt13.wr = 30 * rpm;
    command(3, 2, 0);
    near(chassis.vx, 1);
    near(chassis.vy, 0);
    near(chassis.wr_input, 30 * rpm);
    test_active = true;
    vt13.vx = 0;
    run();
    near(chassis.vx, 3);
    near(chassis.wr_input, (90 - 6 * std::hypot(3.0f, 2.0f)) * rpm);
    vt13.wr = 0;

    // 普通导航断联：摇杆平移也归零，底盘原地 90 rpm
    vt13.vy = -1;
    now_us += 101000;
    run();
    CHECK(!nav.is_connect && chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr_input, 90 * rpm);
    vt13.vy = 0;
    run();
    near(chassis.vy, 0);
    vt13.vy = -1;
    yaw1.is_connect = false;
    run();
    near(chassis.vy, 0);
    yaw1.is_connect = true;
    command(1, 2, 1);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    near(chassis.vx, 1);
    near(chassis.vy, 1);
    yaw2.is_connect = false;
    run();
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr_input, 0);
    yaw2.is_connect = true;
    // 过洞断联：保存过洞状态，平移和旋转均归零；后续普通帧恢复小陀螺
    now_us += 101000;
    run();
    CHECK(!nav.is_connect && is_nav_through_hole);
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr_input, 0);
    command(0, 0, 0);
    CHECK(!is_nav_through_hole);
    near(chassis.wr_input, 84 * rpm);
    vt13.vy = 0;
    test_enable = false;
    run();
    CHECK(!chassis.enabled);
    test_enable = true;
    test_active = false;
    run();
    near(chassis.vx, 0);
    near(chassis.wr_input, 0);
    // 超电规则：裁判离线、C 挡关闭，自检和倒计时充电，遥控使能或比赛中放电补偿
    referee.is_connect = true;
    vt13.is_enable = false;
    vt13.rc.mode = VT13::S;
    for (const int progress : {Referee::REFEREE_SELF_CHECK, Referee::COUNTDOWN_5SEC}) {
        referee.game_status.game_progress = progress;
        run();
        CHECK(super_cap.is_enable && !super_cap.is_boost_enable);
    }
    vt13.is_enable = true;
    run();
    CHECK(super_cap.is_enable && super_cap.is_boost_enable);
    vt13.rc.mode = VT13::C;
    run();
    CHECK(!super_cap.is_enable && !super_cap.is_boost_enable);
    vt13.rc.mode = VT13::S;
    referee.is_connect = false;
    run();
    CHECK(!super_cap.is_enable && !super_cap.is_boost_enable);
    CHECK(irq_depth == 0);
    std::printf("Production chassis flow: %d checks passed\n", checks);
}
