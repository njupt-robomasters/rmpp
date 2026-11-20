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

    // 小 yaw 左转 90°：大 yaw 系向前在云台系中为向右
    gimbal.yaw2.measure = 90 * deg;
    gimbal.yaw.ecd.measure = 30 * deg;
    command(1, 0, 0);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    near(chassis.vx, 0);
    near(chassis.vy, -1);
    near(chassis.wr, 0);
    near(chassis.gimbal_yaw, 30 * deg);

    // 两次坐标转换应等于直接按大 yaw 相对底盘角转换
    for (int i = -10; i <= 10; i++) {
        const float large_yaw = i * 0.2f;
        gimbal.yaw2.measure = i * 0.06f;
        gimbal.yaw.ecd.measure = large_yaw + float(gimbal.yaw2.measure);
        command(1.2f, -0.8f, 0);
        const auto [actual_x, actual_y] = rotate(chassis.vx, chassis.vy, chassis.gimbal_yaw);
        const auto [expected_x, expected_y] = rotate(1.2f, -0.8f, large_yaw);
        near(actual_x, expected_x);
        near(actual_y, expected_y);
    }

    gimbal.yaw2.measure = 0;
    command(0, 0, 1);
    CHECK(chassis.mode == ChassisMatrix::DETACH_MODE);
    near(chassis.wr, WR_MAX);
    command(5, 0, 1);
    near(chassis.wr, WR_MIN);
    command(100, 0, 1);
    near(chassis.vx, VXY_MAX);
    near(chassis.wr, WR_MIN);
    command(0, 0, 0);
    CHECK(chassis.mode == ChassisMatrix::FOLLOW_MODE);
    near(chassis.wr, 0);

    // 人工控制接管全部导航速度和旋转
    vt13.is_connect = true;
    vt13.x = 0.2f;
    command(3, 2, 1);
    near(chassis.vx, 1);
    near(chassis.vy, 0);
    near(chassis.wr, 0);
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
    command(1, 2, 0);
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
