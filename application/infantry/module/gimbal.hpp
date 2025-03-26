#pragma once

#include "m6020.hpp"
#include "m2006.hpp"
#include "pid.hpp"
#include "imu.hpp"
#include "unit.hpp"

class Gimbal {
public:
    enum mode_e {
        ECD_MODE,
        IMU_MODE
    };

    // 向上运动 为 pitch角度增大方向
    // 逆时针旋转 为 yaw角度增大方向
    struct {
        struct {
            Angle absolute{}, relative{}, imu_mode{};
        } pitch{}, yaw{};

        struct {
            Speed speed{};
            float freq = 0;
        } shoot{};
    } ref{}, measure{};

    Gimbal(const IMU &imu, PID::param_t &pitch_pid, PID::param_t &yaw_pid, PID::param_t &shoot_pid);

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void WaitReady();

    void SetEnable(bool is_enable);

    void SetMode(mode_e yaw_mode);

    void SetCurrentAsTarget();

    void SetAngle(float pitch_degree, float yaw_degree);

    void AddAngle(float pitch_degree, float yaw_degree);

    void SetYawSpeedFF(const Speed &yaw_speed_ff);

    void SetPrepareShoot(bool is_prepare_shoot);

    void SetShoot(bool is_shoot, float shoot_freq = 0);

    void Update();

private:
    static constexpr float PITCH_MID = 150.0f; // pitch水平时的绝对角度【单位：角度】
    static constexpr float PITCH_MIN = 124.0f; // pitch最小绝对角度【单位：角度】
    static constexpr float PITCH_MAX = 170.0f; // pitch最大绝对角度【单位：角度】
    static constexpr float YAW_OFFSET = 0.0f; // yaw重合于底盘正方向时的绝对角度【单位：角度】

    static constexpr float SHOOT_NUM_PER_ROUND = 8.0f; // 拨弹电机每转一圈发射的弹数

    const IMU &imu; // 对陀螺仪的引用，用于云台IMU闭环模式

    bool is_enable = false; // 云台使能标志
    mode_e mode = IMU_MODE; // 云台模式
    Speed yaw_speed_ff{}; // yaw速度前馈（小陀螺模式需要）
    float shoot_freq = 0; // 射频
    bool is_prepare_shoot = false; // 摩擦轮状态
    bool is_shoot = false; // 开火标志

    // 电机对象
    M6020 m_pitch, m_yaw;
    M2006 m_shoot;

    void sendCANCmd() const;
};
