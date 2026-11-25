#include "chassis.cpp"

static void run() {
    now_us += 1000;
    nav.update();
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
    near(chassis.wr, 0);

    // 雷达朝前、云台朝左：头部系向前 1 m/s，应在车身系向左 1 m/s
    gimbal.yaw2.measure = 90 * deg;
    gimbal.yaw.ecd.measure = 90 * deg;
    command(1, 0, 1);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    const auto [body_vx, body_vy] = rotate(chassis.vx, chassis.vy, chassis.gimbal_yaw);
    near(body_vx, 0);
    near(body_vy, 1);
    near(chassis.wr, 0);
    near(chassis.gimbal_yaw, 90 * deg);

    // 两轴怎样分担总角，都应将同一条头部系指令原样交给底盘
    for (int i = -10; i <= 10; i++) {
        const float large_yaw = i * 0.2f;
        gimbal.yaw2.measure = i * 0.06f;
        gimbal.yaw.ecd.measure = large_yaw + float(gimbal.yaw2.measure);
        command(1.2f, -0.8f, 1);
        near(chassis.vx, 1.2f);
        near(chassis.vy, -0.8f);
        near(chassis.gimbal_yaw, gimbal.yaw.ecd.measure);
    }

    gimbal.yaw2.measure = 0;
    command(0, 0, 0);
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.wr, 120 * rpm);
    vt13.vy = -2;
    command(1.5f, 0, 0);
    near(chassis.wr, 90 * rpm);
    vt13.vy = 0;
    command(5, 0, 0);
    near(chassis.wr, 60 * rpm);
    command(100, 0, 0);
    near(chassis.vx, vt13.config.vxy_max);
    near(chassis.wr, WR_MIN);
    command(0, 0, 1);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    near(chassis.wr, 0);

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

    // 同向相加后按每轴上限限幅；反向等速抵消，小陀螺按合成平移速度调速
    vt13.vx = 2;
    vt13.vy = -2;
    command(4, -4, 0);
    near(chassis.vx, vt13.config.vxy_max);
    near(chassis.vy, -vt13.config.vxy_max);
    near(chassis.wr, WR_MIN);
    command(-2, 2, 0);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr, vt13.config.wr_max);
    vt13.vx = vt13.vy = 0;

    // 人工模式取遥控速度，切回自主模式恢复导航
    test_active = false;
    vt13.vx = 1;
    command(3, 2, 0);
    near(chassis.vx, 1);
    near(chassis.vy, 0);
    near(chassis.wr, 0);
    test_active = true;
    vt13.vx = 0;
    run();
    near(chassis.vx, 3);
    CHECK(chassis.wr > 0);

    // 导航断开：自动旋转停止，摇杆向右仍然右移，松杆停车
    vt13.vy = -1;
    now_us += 101000;
    run();
    CHECK(!nav.is_connect && chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.vx, 0);
    near(chassis.vy, -1);
    near(chassis.wr, 0);
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
    near(chassis.wr, 0);
    yaw2.is_connect = true;
    vt13.vy = 0;
    test_enable = false;
    run();
    CHECK(!chassis.enabled);
    test_enable = true;
    test_active = false;
    run();
    near(chassis.vx, 0);
    near(chassis.wr, 0);
    CHECK(irq_depth == 0);
    std::printf("Production chassis flow: %d checks passed\n", checks);
}
