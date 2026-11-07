#pragma once

#include "gimbal.hpp"
#include "motor/motor.hpp"

class GimbalClassic : public Gimbal {
  public:
    // 云台电机
    struct Motors {
        Motor &yaw, &pitch;
    } motor;

    GimbalClassic(Attitude& attitude, const Config& config, const Motors& motor);

    void load_yaw_offset(KvStore& kv_store) override;

    void save_yaw_offset(KvStore& kv_store) override;

    void set_yaw_zero() override;

  private:
    void set_motors_enable(bool is_enable) override;

    void update_motors() override;

    // 角度正解：电机角度 -> 云台姿态
    void forward() override;

    // 角度正解：云台姿态 -> 电机角度
    void backward() override;
};
