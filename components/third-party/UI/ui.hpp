#pragma once

#include <cstdint>
class UI{

public:
    void Init_UI(uint8_t robot_id);// 初始化UI
    void Update_UI(); //更新UI

    //以下为并发更新的变量：
    uint8_t set_bullet_frequency = 10; //弹丸发射频率
    uint8_t set_gimbal_is_imu_mode = 1; //云台模式，是imu模式为1，否则为0
    uint8_t set_chassis_max_speed = 1; //底盘平移速度
    uint8_t set_is_locked = 0; //自瞄是否锁定
    uint8_t set_is_firing = 0; //是否自动火控
    uint8_t set_is_turning = 0; //是否开启小陀螺
    uint16_t set_chassis_vr = 0;//底盘自转速度
    uint8_t set_center_gain_status = 0;//中心增益点占领情况

private:
};