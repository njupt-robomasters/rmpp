#include "attitude.hpp"

Attitude::Attitude(Imu& imu)
    : imu_(imu) {}

void Attitude::update() {
    imu_.update();
    solve(imu_.gyro, imu_.accel);

    // 四元数 -> 欧拉角
    const Angle last_yaw = yaw;
    const auto euler = quaternion_to_euler(q);
    yaw = euler.yaw;
    pitch = euler.pitch;
    roll = euler.roll;
    yaw_total_angle += Angle(yaw - last_yaw);
}
