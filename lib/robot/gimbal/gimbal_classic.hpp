#pragma once

#include "gimbal.hpp"

// 单 yaw、单 pitch 云台
class GimbalClassic : public Gimbal {
  public:
    // 云台电机
    struct Motors {
        Motor &yaw, &pitch;
    };

    GimbalClassic(const Motors& motor, const Config& config);

    void set_yaw_zero() override;

    void load_yaw_offset(KvStore& kv_store) override;
    void save_yaw_offset(KvStore& kv_store) override;

  public:
    Motors motor;

  private:
    void set_motors_enable(bool is_enable) override;
    void update_motors() override;
    void forward() override;
    void backward(float dt) override;
};
