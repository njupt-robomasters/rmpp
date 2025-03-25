#pragma once

#include "m6020.hpp"
#include "dm4310.hpp"
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
            Angle absolute{};
        } shoot{};
    } ref{}, measure{};

    Gimbal(const IMU &imu, PID::param_t &pitch_pid, PID::param_t &yaw_pid, PID::param_t &shoot_pid);

    void Init();

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void ResetReady();

    [[nodiscard]] bool CheckReady() const;

    void SetEnable(bool is_enable);

    void SetMode(mode_e yaw_mode);

    void SetCurrentAsTarget();

    void SetAngle(float pitch_degree, float yaw_degree);

    void AddAngle(float pitch_degree, float yaw_degree);

    void SetYawSpeedFF(const Speed &yaw_speed_ff);

    void SetPrepareShoot(bool is_prepare_shoot);

    void Shoot();

    void Update();

private:
    static constexpr float PITCH_MID = 150.0f; // pitch水平时的绝对角度【单位：角度】
    static constexpr float PITCH_MIN = 124.0f; // pitch最小绝对角度【单位：角度】
    static constexpr float PITCH_MAX = 170.0f; // pitch最大绝对角度【单位：角度】
    static constexpr float YAW_OFFSET = 0.0f; // yaw重合于底盘正方向时的绝对角度【单位：角度】

    static constexpr float SHOOT_NUM_PER_ROUND = 6.0f; // 拨弹电机每转一圈发射的弹数

    const IMU &imu; // 对陀螺仪的引用，用于云台IMU闭环模式

    bool is_enable = false; // 云台使能标志
    mode_e mode = IMU_MODE; // 云台模式
    Speed yaw_speed_ff{}; // yaw速度前馈（小陀螺模式需要）
    bool is_prepare_shoot = false; // 摩擦轮状态

    uint32_t can_send_cnt = 0; // 用于间隔发送三个电机的CAN报文

    // 电机对象
    DM4310 m_pitch;
    M6020 m_yaw;
    DM4310 m_shoot;

    void sendCANCmd();
};
