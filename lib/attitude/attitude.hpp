#pragma once

#include "bsp/imu.hpp"
#include "math/angle.hpp"

// 姿态解算父类：读取 Imu，由子类算法更新四元数，统一输出欧拉角和累计 yaw
class Attitude {
  public:
    Quaternionf q = Quaternionf::Identity(); // 机体到世界
    Angle yaw, pitch, roll;                  // Z-Y-X 欧拉角，rad
    float yaw_total_angle = 0.0f;            // 累积偏航角，rad，不归一化

    explicit Attitude(Imu& imu);

    // 更新 Imu 并推进一步解算；Imu 读取失败时使用其上次输出
    void update();

  protected:
    // 用机体系 gyro（rad/s）、accel（m/s²）推进一步并更新 q
    virtual void solve(const Vector3f& gyro, const Vector3f& accel) = 0;

  private:
    Imu& imu_;
};
