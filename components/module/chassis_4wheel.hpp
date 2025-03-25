#pragma once

#include "m3508.hpp"
#include "pid.hpp"
#include "unit.hpp"

class Chassis4Wheel {
public:
    enum chassis_type_e {
        OMNI, // 全向轮
        MECANUM // 麦克纳姆轮
    };

    struct {
        // vx 左右平移速度【单位：m/s】
        // vy 前进后退速度【单位：m/s】
        // vz 旋转线速度，逆时针为正【单位：m/s】
        // vr 旋转角速度，逆时针为正
        struct {
            float vx = 0, vy = 0, vz = 0;
            Speed vr{};
        } gimbal, chassis;

        // 轮子速度【单位：m/s】
        struct {
            float v1 = 0, v2 = 0, v3 = 0, v4 = 0;
        } wheel;
    } ref{}, measure{};

    Chassis4Wheel(chassis_type_e chassis_type, float wheel_radius, float chassis_radius, PID::param_t &wheel_pid_param);

    void ParseCAN(uint32_t id, uint8_t data[8]);

    void ResetReady();

    [[nodiscard]] bool CheckReady() const;

    void SetEnable(bool is_enable);

    void SetSpeed(float vx, float vy, float vr_rpm);

    void SetGimbalAngle_RefByChassis(const Angle &gimbal_angle_refBy_chassis);

    void SetPowerLimit(float power);

    void Update();

private:
    chassis_type_e chassis_type;
    float wheel_radius, chassis_radius;
    float wheel_perimeter, chassis_perimeter;

    // 底盘使能标志
    bool is_enable = false;

    // 设置云台参考于底盘的角度，逆时针为正方向
    Angle gimbal_angle_refBy_chassis{};

    // 电机对象
    M3508 m1, m2, m3, m4;

    // 底盘功率控制
    float power_estimate = 0; // 当前功率估计
    float power_limit = 120; // 功率限制
    float current_max = 0;
    float current_ratio = 1; // 电流衰减系数

    void estimatePower();

    void calcCurrentRatio();

    void forwardCalc();

    void inverseCalc();

    void sendCurrentCmd() const;
};
