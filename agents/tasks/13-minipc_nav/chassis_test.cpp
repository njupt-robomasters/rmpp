#include "chassis.cpp"

static void run() {
    now_us += 1000;
    nav.update();
    chassis_loop();
}

static void command(const float vx, const float vy, const uint8_t flag) {
    const auto bytes = make_frame(vx, vy, 0.7f, flag);
    feed(2, bytes.data(), bytes.size());
    run();
}

int main() {
    run();
    CHECK(chassis.enabled && chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr, 0);

    // 导航速度直接使用头坐标系，小 yaw 角度不改变输入
    gimbal.yaw2.measure = 90 * deg;
    gimbal.yaw.ecd.measure = 30 * deg;
    command(1, 0, 1);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    near(chassis.vx, 1);
    near(chassis.vy, 0);
    near(chassis.wr, 0);
    near(chassis.gimbal_yaw, 30 * deg);

    gimbal.yaw2.measure = 0;
    command(0, 0, 0);
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.wr, NAV_WR);
    command(5, 0, 0);
    near(chassis.wr, NAV_WR);
    command(100, 0, 0);
    near(chassis.vx, VXY_MAX);
    near(chassis.wr, NAV_WR);
    command(0, 0, 1);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    near(chassis.wr, 0);

    // 人工平移接管速度，保留 flag 对应的小陀螺或随头
    vt13.is_connect = true;
    vt13.x = 0.2f;
    command(3, 2, 0);
    near(chassis.vx, 1);
    near(chassis.vy, 0);
    near(chassis.wr, NAV_WR);
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    command(3, 2, 1);
    near(chassis.vx, 1);
    near(chassis.wr, 0);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    command(3, 2, 0);
    vt13.wheel = 0.5f;
    run();
    near(chassis.wr, 0.5f * WR_MAX);
    vt13.wheel = 0;
    vt13.x = 0;
    run();
    near(chassis.vx, 3);
    CHECK(chassis.wr > 0);

    // 超时在 fn 自主测试模式下也撤销随头和小陀螺
    now_us += 101000;
    run();
    CHECK(!nav.is_connect && chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.vx, 0);
    near(chassis.vy, 0);
    near(chassis.wr, 0);
    // 导航离线时 Fn 和人工平移保持分离模式
    vt13.x = 0.2f;
    run();
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.vx, 1);
    near(chassis.wr, 0);
    vt13.x = 0;
    command(1, 2, 1);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    yaw2.is_connect = false;
    run();
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.vx, 0);
    near(chassis.wr, 0);
    yaw2.is_connect = true;
    test_enable = false;
    run();
    CHECK(!chassis.enabled);
    near(chassis.vx, 0);
    test_enable = true;
    test_active = false;
    test_fn = false;
    run();
    near(chassis.vx, 0);
    near(chassis.wr, 0);
    CHECK(irq_depth == 0);
    std::printf("Production chassis flow: %d checks passed\n", checks);
}
