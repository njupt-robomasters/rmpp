#pragma once

#include "gimbal.hpp"

// 大小 yaw 串联云台：大 yaw 直接跟随目标，小 yaw 补偿大 yaw 未到位的部分
class GimbalDualYaw : public Gimbal {
  public:
    // 大小yaw分配
    struct {
        Angle ref, measure;
    } yaw1, yaw2; // yaw1-大yaw，yaw2-小yaw

  private:
    struct Motors {
        Motor &yaw1, &yaw2, &pitch; // yaw1-大yaw，yaw2-小yaw
    } motor;

  public:
    GimbalDualYaw(const Motors& motor, const Config& config);

    void set_yaw_zero() override;

    void load_yaw_offset(KvStore& kv_store) override;

    void save_yaw_offset(KvStore& kv_store) override;

  private:
    void set_motors_enable(bool is_enable) override;

    void update_motors() override;

    void forward() override;

    void backward(float dt) override;
};
