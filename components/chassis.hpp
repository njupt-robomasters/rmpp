#pragma once

#include "m3508.hpp"
#include "pid.hpp"
#include "unit.hpp"

class Chassis {
public:
    struct {
        struct {
            float chassis = 0, gimbal = 0;
        } vx{}, vy{}; // vx 左右平移速度【单位：m/s】，vy 前进后退速度【单位：m/s】
        float vz = 0; // vz 旋转线速度，逆时针为正【单位：m/s】
        Speed vr{}; // vr 旋转角速度，逆时针为正
        float v1 = 0, v2 = 0, v3 = 0, v4 = 0; // 轮子速度【单位：m/s】
    } ref{}, measure{};

    // 底盘功率估计
    struct {
        float raw = 0, lpf = 0;
    } power_estimate{};

    Chassis(const PID::pid_param_t &wheel_pid_param, const PID::pid_param_t &speed_comp_pid_param,
            const PID::pid_param_t &power_limit_pid_param);

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void ResetReady();

    [[nodiscard]] bool CheckReady() const;

    void SetEnable(bool is_enable);

    void SetSpeed(float vx, float vy, float vr_rpm);

    // 设置云台参考于底盘的角度，逆时针为正方向
    void SetGimbalAngle_RefByChassis(const Angle &gimbal_angle_refBy_chassis);

    void SetPowerLimit(float power);

    void Update();

private:
    // 底盘结构参数
    static constexpr float WHEEL_RADIUS = 0.063f; // 轮子半径【单位：m】
    static constexpr float CHASSIS_RADIUS = 0.225f; // 底盘半径【单位：m】

    // 底盘结构参数（自动计算）
    static constexpr float WHEEL_PERIMETER = 2 * static_cast<float>(M_PI) * WHEEL_RADIUS; // 轮子周长【单位：m】
    static constexpr float CHASSIS_PERIMETER = 2 * static_cast<float>(M_PI) * CHASSIS_RADIUS; // 底盘周长【单位：m】

    static constexpr float SPEED_COMP_MAX = 1.0f; // 最大补偿速度【单位：m/s】

    static constexpr float ESTIMATE_PWOER_LPF_FREQ = 10.0f; // 底盘功率估计滤波截止频率【单位：H】

    bool is_enable = false;

    // 原定底盘前进方向->云台方向 的角度（逆时针为正）
    Angle gimbal_angle_refBy_chassis{};

    // 速度补偿
    // （暂时未用到，轮子PID给的激进后，小陀螺模式前进已经很稳）
    float vx_comp = 0, vy_comp = 0;
    PID vx_comp_pid, vy_comp_pid;

    // 用于底盘功率估计滤波
    uint32_t dwt_cnt = 0;
    float dt = 0;

    // 底盘功率控制
    float power_limit = 120;
    float power_comp = 0;
    float speed_ratio = 1;
    PID power_limit_pid;

    // 电机对象
    M3508 m1, m2, m3, m4;

    void calcPowerLimit();

    void calcSpeedComp();

    void forwardCalc();

    void inverseCalc();

    void sendCurrentCMD() const;

    void powerEstimate();
};
