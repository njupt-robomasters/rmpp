#include "chassis.hpp"
#include "app.hpp"

// 操控参数
static constexpr float VX_MAX = 3.0f;        // 前向速度上限，m/s
static constexpr float WR_MAX = 180 * deg_s; // 航向角速度上限，rad/s

static uint32_t send_count = 0; // CAN2 分时发送计数

void chassis_loop() {
    attitude.update();

    // FSi6X：SWA 下使能，SWB 下允许自救，SWC 选腿长，SWD 下跳跃；断联时拨杆为 ERR、摇杆为 0
    chassis.set_enable(fsi6x.is_connect && fsi6x.swa == FSi6X::DOWN);
    chassis.set_rescue(fsi6x.swb == FSi6X::DOWN);
    if (fsi6x.is_connect) {
        chassis.set_jump(fsi6x.swd == FSi6X::DOWN);
    }

    // 右摇杆：竖直 x 前推为正对应前进，水平 y 左推为正对应逆时针转向
    chassis.set_speed(fsi6x.x * VX_MAX, fsi6x.y * WR_MAX);

    if (fsi6x.swc == FSi6X::UP)
        chassis.set_leg_length(0.14f);
    else if (fsi6x.swc == FSi6X::MID)
        chassis.set_leg_length(0.22f);
    else if (fsi6x.swc == FSi6X::DOWN)
        chassis.set_leg_length(0.33f);

    super_cap.update();

    // CAN2 左右关节交替发送，每个关节 500 Hz；每周期不超过 3 帧，匹配 3 个发送邮箱
    if (send_count++ % 10 == 0) { // 超电 100 Hz 发送
        super_cap.send();
    } else if (send_count % 2 == 0) {
        // 姿态来自本板 IMU；换用虚拟 IMU 时传入换算后的底盘 q 与 gyro
        chassis.update(attitude.q, imu.gyro);

        left_front.send();
        left_back.send();
        CanFrame frame{.id = 0x200, .size = 8};
        left_wheel.write_can_frame(frame);
        right_wheel.write_can_frame(frame);
        Can(2).send(frame);
    } else {
        right_back.send();
        right_front.send();
    }
}
