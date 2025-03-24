#pragma once

#include "m3508.hpp"
#include "pid.hpp"
#include "unit.hpp"

namespace Infantry {
    class Chassis {
    public:
        struct {
            // vx 左右平移速度【单位：m/s】
            // vy 前进后退速度【单位：m/s】
            // vz 旋转线速度，逆时针为正【单位：m/s】
            // vr 旋转角速度，逆时针为正
            struct {
                float vx = 0, vy = 0, vz = 0;
                Speed vr{};
            } gimbal, correct, chassis;

            // 轮子速度【单位：m/s】
            struct {
                float v1 = 0, v2 = 0, v3 = 0, v4 = 0;
            } wheel;
        } ref{}, measure{};

        Chassis(PID::param_t &wheel_pid_param, PID::param_t &speed_comp_pid_param);

        void ParseCAN(uint32_t id, uint8_t data[8]);

        void ResetReady();

        [[nodiscard]] bool CheckReady() const;

        void SetEnable(bool is_enable);

        void SetSpeed(float vx, float vy, float vr_rpm);

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

        // 底盘使能标志
        bool is_enable = false;

        // 设置云台参考于底盘的角度，逆时针为正方向
        Angle gimbal_angle_refBy_chassis{};

        // 电机对象
        M3508 m1, m2, m3, m4;

        // 速度补偿
        float vx_comp = 0, vy_comp = 0;
        PID vx_comp_pid, vy_comp_pid;

        // 底盘功率控制
        float power_estimate = 0; // 当前功率估计
        float power_limit = 120; // 功率限制
        float current_max = 0;
        float current_ratio = 1; // 电流衰减系数

        void estimatePower();

        void calcCurrentRatio();

        void forwardCalc();

        void inverseCalc();

        void sendCurrentCmd();
    };
}
