#include "app.hpp"

// 操控参数
static constexpr float VX_MAX = 3.0f;        // 前向速度上限，m/s
static constexpr float WR_MAX = 180 * deg_s; // 航向角速度上限，rad/s

// 跨周期状态：拨杆异常时保持上次腿长
static float leg_length = 0.14f;

void chassis_loop() {
    // FSi6X：SWA 下使能，SWB 下允许自救，SWC 选腿长，SWD 下跳跃；断联时拨杆为 ERR、摇杆为 0
    chassis.set_enable(fsi6x.is_connect && fsi6x.swa == FSi6X::DOWN);
    chassis.set_rescue(fsi6x.swb == FSi6X::DOWN);
    chassis.set_jump(fsi6x.swd == FSi6X::DOWN);

    // 右摇杆：竖直 x 前推为正对应前进，水平 y 左推为正对应逆时针转向
    chassis.set_speed(fsi6x.x * VX_MAX, fsi6x.y * WR_MAX);

    if (fsi6x.swc == FSi6X::UP)
        leg_length = 0.14f;
    else if (fsi6x.swc == FSi6X::MID)
        leg_length = 0.22f;
    else if (fsi6x.swc == FSi6X::DOWN)
        leg_length = 0.33f;
    chassis.set_leg_length(leg_length);

    // 姿态来自本板 IMU；换用虚拟 IMU 时传入换算后的底盘 q 与 gyro
    chassis.update(attitude.q, imu.gyro);
}
