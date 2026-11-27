#include "chassis.cpp"
#include "minipc.cpp"
#include "sentry.cpp"

static void run() {
    now_us += 1000;
    vt13.rc.fn = test_active;
    vt13.rc.pause = !test_active;
    minipc_loop();
    chassis_loop();
}

static void command(const float vx, const float vy) {
    const auto bytes = make_frame(vx, vy, 0.7f, 0, 0, 0);
    feed(2, bytes.data(), bytes.size());
    run();
}

int main() {
    run();
    CHECK(chassis.enabled && chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr_input, vt13.config.wr_max);

    // 雷达朝前（yaw1 = 0）、云台朝左（yaw2 = +90°）：导航向前 1 m/s 在头部系为向右 1 m/s，落到车身系为向前 1 m/s
    gimbal.yaw2.measure = 90 * deg;
    gimbal.yaw.ecd_measure = 90 * deg;
    command(1, 0);
    near(chassis.vy, -1);
    const auto [body_vx, body_vy] = rotate(chassis.vx, chassis.vy, chassis.gimbal_yaw);
    near(body_vx, 1);
    near(body_vy, 0);
    near(chassis.wr_input, vt13.config.wr_max);
    near(chassis.gimbal_yaw, 90 * deg);

    // 同一姿态下遥控向前 1 m/s 仍沿头部朝向（车身向左），与导航向前合成后车身为前 1、左 1
    vt13.vx = 1;
    command(1, 0);
    const auto [mix_body_vx, mix_body_vy] = rotate(chassis.vx, chassis.vy, chassis.gimbal_yaw);
    near(mix_body_vx, 1);
    near(mix_body_vy, 1);
    vt13.vx = 0;

    // 任意小 yaw 偏转下，纯导航速度经两级旋转后始终等于仅按大 yaw 转到车身系的结果
    for (int i = -10; i <= 10; i++) {
        const float large_yaw = i * 0.2f;
        gimbal.yaw2.measure = i * 0.06f;
        gimbal.yaw.ecd_measure = large_yaw + float(gimbal.yaw2.measure);
        command(1.2f, -0.8f);
        const auto [actual_bx, actual_by] = rotate(chassis.vx, chassis.vy, chassis.gimbal_yaw);
        const auto [expect_bx, expect_by] = rotate(1.2f, -0.8f, large_yaw);
        near(actual_bx, expect_bx);
        near(actual_by, expect_by);
        near(chassis.gimbal_yaw, gimbal.yaw.ecd_measure);
    }

    // 平移不另行限幅，小陀螺固定 90 rpm
    gimbal.yaw2.measure = 0;
    command(100, 0);
    near(chassis.vx, 100);
    near(chassis.vy, 0);
    near(chassis.wr_input, vt13.config.wr_max);

    // 导航向前、摇杆向左：两个方向都有速度，松杆后只剩导航
    vt13.vy = -2;
    command(2, 0);
    near(chassis.vx, 2);
    near(chassis.vy, -2);
    vt13.vy = 0;
    run();
    near(chassis.vx, 2);
    near(chassis.vy, 0);

    // 同向相加保留超出遥控满量程的平移目标；反向抵消归零
    vt13.vx = 2;
    vt13.vy = -2;
    command(4, -4);
    near(chassis.vx, 6);
    near(chassis.vy, -6);
    command(-2, 2);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr_input, vt13.config.wr_max);
    vt13.vx = vt13.vy = 0;

    // 人工模式取遥控速度，切回自主恢复导航
    test_active = false;
    vt13.vx = 1;
    vt13.wr = 30 * rpm;
    command(3, 2);
    near(chassis.vx, 1);
    near(chassis.vy, 0);
    near(chassis.wr_input, 30 * rpm);
    test_active = true;
    vt13.vx = 0;
    vt13.wr = 0;
    run();
    near(chassis.vx, 3);
    near(chassis.vy, 2);
    near(chassis.wr_input, vt13.config.wr_max);

    // 导航断联：只剩遥控平移，小陀螺 90 rpm
    vt13.vy = -1;
    now_us += 101000;
    run();
    CHECK(!nav.is_connect);
    near(chassis.vx, 0);
    near(chassis.vy, -1);
    near(chassis.wr_input, vt13.config.wr_max);
    vt13.vy = 0;

    // yaw 反馈丢失不影响速度指令
    command(1, 2);
    yaw1.is_connect = false;
    run();
    near(chassis.vx, 1);
    near(chassis.vy, 2);
    near(chassis.wr_input, vt13.config.wr_max);
    yaw1.is_connect = true;
    yaw2.is_connect = false;
    run();
    near(chassis.vx, 1);
    near(chassis.vy, 2);
    yaw2.is_connect = true;
    command(0, 0);

    // 正赛仍叠加遥控平移
    referee.game_status.game_progress = Referee::GAMING;
    vt13.vx = 2;
    command(1, 0);
    near(chassis.vx, 3);
    near(chassis.wr_input, vt13.config.wr_max);
    vt13.vx = 0;

    // C 挡优先失能，N 挡使能
    for (const int progress : {Referee::NOT_STARTED, Referee::REFEREE_SELF_CHECK, Referee::COUNTDOWN_5SEC, Referee::GAMING}) {
        referee.game_status.game_progress = progress;
        vt13.rc.mode = VT13::C;
        vt13.is_enable = false;
        run();
        CHECK(!chassis.enabled);
        vt13.rc.mode = VT13::N;
        vt13.is_enable = true;
        run();
        CHECK(chassis.enabled);
    }

    // 超电随遥控使能
    vt13.is_enable = false;
    run();
    CHECK(!super_cap.is_enable && !super_cap.is_boost_enable);
    vt13.is_enable = true;
    run();
    CHECK(super_cap.is_enable && super_cap.is_boost_enable);
    CHECK(irq_depth == 0);
    std::printf("Production chassis flow: %d checks passed\n", checks);
}
