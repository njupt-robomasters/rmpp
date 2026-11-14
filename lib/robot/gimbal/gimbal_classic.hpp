#pragma once

#include "gimbal.hpp"

// 单 yaw、单 pitch 云台
class GimbalClassic : public Gimbal {
  private:
    struct Motors {
        Motor &yaw, &pitch;
    } motor;

  public:
    GimbalClassic(const Motors& motor, const Config& config);

  private:
    void set_motors_enable(bool is_enable) override;

    void update_motors() override;

    void forward() override;

    void backward(float dt) override;
};
