#pragma once

#include "m6020.hpp"
#include "m2006.hpp"
#include "imu.hpp"

class Gimbal {
public:
    enum yaw_mode_t {
        ECD_MODE,
        IMU_MODE
    };

    // 向上运动 为 pitch角度增大方向
    // 逆时针旋转 为 yaw角度增大方向

    // 云台设定值
    float pitch_angle_set = 0, yaw_angle_set = 0, shoot_freq_set = 0; // 人类友好值
    float pitch_angle_absolute_set = 0, yaw_angle_absolute_set = 0, shoot_v_tps_set = 0; // 对接电机控制值

    // 云台实际值
    float pitch_angle = 0, yaw_angle = 0, shoot_freq = 0; // 人类友好值
    float pitch_angle_absolute = 0, yaw_angle_absolute = 0, shoot_speed_tps = 0; // 对接电机控制值

    Gimbal(const IMU &imu, const mit_t &pitch_mit, const mit_t &yaw_mit, const mit_t &shoot_mit);

    void Init();

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void SetEnable(bool is_enable);

    void SetYawMode(yaw_mode_t yaw_mode);

    void SetCurrentAsTarget();

    void SetAngle(float pitch_angle_set, float yaw_angle_set);

    void AddAngle(float pitch_angle_add, float yaw_angle_add);

    void SetShootFreq(float shoot_freq_set);

    void SetYAW_V_APS_FF(float yaw_v_aps_ff);

    void Update();

private:
    static constexpr int PITCH_RESET_MS = 1500; // pitch回零时间【单位：毫秒】
    static constexpr float PITCH_RANGE = 50.0f; // PITCH运动范围【单位：角度】
    static constexpr int YAW_OFFSET = 0.0f; // yaw相对于前进方向的偏移【单位：角度】
    static constexpr float SHOOT_NUM_PER_ROUND = 8.0f; // 拨弹电机每转一圈发射的弹数

    // 构造函数参数
    const IMU &imu; // 对陀螺仪的引用，用于云台yaw的IMU闭环模式
    yaw_mode_t yaw_mode = ECD_MODE;

    bool is_enable = false;
    float pitch_offset = 0; // pitch行程最低端，有回零函数，此处无需设置【单位：角度】

    float yaw_v_aps_ff = 0;

    M6020 m_pitch, m_yaw;
    M2006 m_shoot;

    void sendCANCmd() const;
};
