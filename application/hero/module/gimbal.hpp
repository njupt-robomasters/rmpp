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

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void WaitReady();

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
    static constexpr float PITCH_MIN = 40.0f; // pitch最小绝对角度【单位：角度】
    static constexpr float PITCH_MID = 60.0f; // pitch水平时的绝对角度【单位：角度】
    static constexpr float PITCH_MAX = 80.0f; // pitch最大绝对角度【单位：角度】
    static constexpr float YAW_OFFSET = 220.676f; // yaw重合于底盘正方向时的绝对角度【单位：角度】

    static constexpr float SHOOT_NUM_PER_ROUND = 6.0f; // 拨弹电机每转一圈发射的弹数

    const IMU &imu; // 对陀螺仪的引用，用于云台IMU闭环模式

    // 云台使能标志
    bool is_enable = false;

    // 云台模式
    mode_e mode = ECD_MODE;

    // yaw速度前馈（小陀螺模式需要）
    Speed yaw_speed_ff{};

    // 摩擦轮状态
    bool is_prepare_shoot = false;

    // 当前射击是否为完成
    bool is_shoot_finish = true;

    // 用于间隔发送三个电机的CAN报文
    uint32_t can_send_cnt = 0;

    // 电机对象
    DM4310 m_pitch;
    M6020 m_yaw;
    DM4310 m_shoot;

    uint32_t stop_shoot_cnt = 0;

    void sendCANCmd();
};
